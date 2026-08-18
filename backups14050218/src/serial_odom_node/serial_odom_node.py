# serial_odom_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial, threading, time, math

PORT = '/dev/turtlebot_arduino'
BAUD = 115200
WHEEL_RADIUS = 0.0625
WHEEL_BASE = 0.50
TICKS_PER_REV = 50
MAX_PWM = 255
# conservative mapping: tune MAX_WHEEL_SPEED to your drivetrain
MAX_WHEEL_SPEED = 0.027  # m/s -> adjust by test

# Scale turning relative to linear speed (smaller = slower turning)
TURNING_GAIN = 0.2  # try 0.2; use 0.1 if still too fast

def yaw_to_quat(y):
    return (0.0, 0.0, math.sin(y/2.0), math.cos(y/2.0))

class SerialOdomNode(Node):
    def __init__(self):
        super().__init__('serial_odom_node')
        self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_brd = TransformBroadcaster(self)
        self.lock = threading.Lock()

        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.last_left = None; self.last_right = None; self.last_time_ms = None

        t = threading.Thread(target=self.read_loop, daemon=True)
        t.start()

    def cmd_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        # Apply turning gain so rotations are not too aggressive
        vl = v - (TURNING_GAIN * w * WHEEL_BASE / 2.0)
        vr = v + (TURNING_GAIN * w * WHEEL_BASE / 2.0)
        pL = int((vl / MAX_WHEEL_SPEED) * MAX_PWM)
        pR = int((vr / MAX_WHEEL_SPEED) * MAX_PWM)
        pL = max(-MAX_PWM, min(MAX_PWM, pL))
        pR = max(-MAX_PWM, min(MAX_PWM, pR))
        line = f'P,{pL},{pR}\n'
        try:
            self.ser.write(line.encode())
        except Exception as e:
            self.get_logger().error('Serial write failed: ' + str(e))

    def read_loop(self):
        while rclpy.ok():
            try:
                raw = self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception:
                continue
            if not raw:
                continue
            parts = raw.split(',')
            if parts[0] == 'E' and len(parts) >= 4:
                try:
                    l = int(parts[1]); r = int(parts[2]); t_ms = int(parts[3])
                except:
                    continue
                self.handle_encoders(l, r, t_ms)

    def handle_encoders(self, l, r, t_ms):
        with self.lock:
            if self.last_left is None:
                self.last_left = l; self.last_right = r; self.last_time_ms = t_ms
                return
            dl = l - self.last_left
            dr = r - self.last_right
            dt = (t_ms - self.last_time_ms) / 1000.0
            if dt <= 0:
                return
            dist_l = (dl / TICKS_PER_REV) * (2.0 * math.pi * WHEEL_RADIUS)
            dist_r = (dr / TICKS_PER_REV) * (2.0 * math.pi * WHEEL_RADIUS)
            ds = (dist_r + dist_l) / 2.0
            dtheta = (dist_r - dist_l) / WHEEL_BASE
            # integrate pose
            self.x += ds * math.cos(self.theta + dtheta/2.0)
            self.y += ds * math.sin(self.theta + dtheta/2.0)
            self.theta += dtheta
            vx = ds / dt
            vth = dtheta / dt
            now = self.get_clock().now().to_msg()

            # publish odom
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            q = yaw_to_quat(self.theta)
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            odom.twist.twist.linear.x = vx
            odom.twist.twist.angular.z = vth
            self.odom_pub.publish(odom)

            # broadcast tf
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_brd.sendTransform(t)

            self.last_left = l; self.last_right = r; self.last_time_ms = t_ms

def main(args=None):
    rclpy.init(args=args)
    node = SerialOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
