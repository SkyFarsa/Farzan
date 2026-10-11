# serial_odom_node.py
#
# Fixes applied vs previous version:
#   1. WHEEL_BASE corrected to 0.48 m (measured: 51cm outer - 3cm wheel thickness)
#   2. TICKS_PER_REV reset to 0 (placeholder) pending new manual calibration —
#      set this to your measured value before running for real
#   3. (unchanged) dt sanity check, thread-safe publishing, reset service,
#      midpoint pose integration, proper covariance
#
# URDF values used:
#   wheel_radius     = 0.06 m   (12cm diameter / 2)
#   wheel_separation = 0.48 m   (measured center-to-center)
#
# ── HOW TO CALIBRATE TICKS_PER_REV ──────────────────────────────────────────
#   1. Flash the Arduino (arduino_firmware.ino) and connect to RPi
#   2. Run this node
#   3. Mark one wheel with a piece of tape
#   4. Rotate that wheel exactly N full revolutions by hand (slowly).
#      Use N=5 or N=10 and divide by N — much more accurate than a single
#      revolution, since it's easy to over/undershoot exactly "one" turn by hand.
#   5. Run:  ros2 topic echo /odom --once
#      or watch raw serial:  screen /dev/turtlebot_arduino 115200
#   6. The change in left_ticks (printed as E,LEFT,RIGHT,TIME) divided by N
#      is your TICKS_PER_REV
#   7. Sanity check: your encoder ISR only counts RISING edges on channel A
#      (single-edge counting, not full x4 quadrature), so this number should
#      land close to 1x the Omron E6A2-CW3C's datasheet PPR for your model.
#      If it's noticeably lower, you may be missing ticks at speed.
#   8. Update TICKS_PER_REV below and restart the node
# ─────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster
import serial
import threading
import math

# ── Serial port ──────────────────────────────────────────────────────────────
PORT = '/dev/turtlebot_arduino'
BAUD = 115200

# ── Robot dimensions (from URDF) ─────────────────────────────────────────────
WHEEL_RADIUS     = 0.06    # metres — wheel_radius in xacro
WHEEL_BASE       = 0.48    # metres — wheel_separation in xacro (measured: 0.51 - 0.03)

# ── Encoder resolution ───────────────────────────────────────────────────────
# IMPORTANT: This is counts per revolution with single-edge (RISING only) counting.
# Do NOT multiply by 4 — the current ISR does not do full quadrature decoding.
# Set to 0 until calibrated; node will warn and skip odometry math if left at 0.
TICKS_PER_REV    = 203     # <-- calibrated: 1 manual revolution, both wheels matched

# ── Motor speed mapping ───────────────────────────────────────────────────────
MAX_PWM          = 255
# Max linear speed of one wheel in m/s at MAX_PWM.
# Start conservative; tune upward once the robot moves straight.
MAX_WHEEL_SPEED  = 0.5     # m/s — adjust by physical test

# ── Turning gain ─────────────────────────────────────────────────────────────
# Scales angular velocity commands. Lower = gentler turns.
# 1.0 is physically correct; reduce if turns are too aggressive.
TURNING_GAIN     = 1.0

# ── Encoder tick-delta plausibility bound ────────────────────────────────────
# Fixes the documented "encoder serial corruption" issue: a corrupted-but-
# still-numeric serial packet (flipped digit, shifted field) can produce a
# tick delta that parses fine as an int but is physically impossible, and it
# gets permanently baked into (x, y) since nothing previously checked it -
# only dt was sanity-checked, never dl/dr themselves. This mirrors that same
# style of check for tick counts instead of time.
#
# Set generously above MAX_WHEEL_SPEED (0.5 m/s) so real motion, even
# somewhat faster than currently configured, is never falsely rejected -
# this is only meant to catch clearly-garbled packets (jumps of hundreds or
# thousands of ticks in one ~50ms update), not to police normal speed.
MAX_PLAUSIBLE_WHEEL_SPEED_MPS = 2.0   # generous ceiling, not a real target speed

# ── Odometry covariance ───────────────────────────────────────────────────────
# These tell SLAM/EKF how much to trust this odometry.
# Larger = less trust. Tune after calibration.
POSE_COVAR_XY    = 0.01    # metres^2
POSE_COVAR_YAW   = 0.05    # radians^2
TWIST_COVAR_VX   = 0.01
TWIST_COVAR_VYAW = 0.05


# Ticks/sec at the generous speed ceiling above - derived once here so the
# per-cycle check below just scales it by that update's actual dt.
_MAX_PLAUSIBLE_TICKS_PER_SEC = (
    MAX_PLAUSIBLE_WHEEL_SPEED_MPS / (2.0 * math.pi * WHEEL_RADIUS)
) * TICKS_PER_REV


def yaw_to_quat(yaw):
    """Convert a yaw angle (radians) to a quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SerialOdomNode(Node):
    def __init__(self):
        super().__init__('serial_odom_node')

        # ── Warn if not calibrated ────────────────────────────────────────
        if TICKS_PER_REV == 0:
            self.get_logger().warn(
                'TICKS_PER_REV is 0 — odometry will not work! '
                'Follow the calibration steps in the file header.'
            )

        # ── Serial ───────────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port {PORT}: {e}')
            raise

        # ── ROS interfaces ────────────────────────────────────────────────
        self.cmd_sub  = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_brd   = TransformBroadcaster(self)

        # Reset odometry service — call with: ros2 service call /reset_odom std_srvs/srv/Empty
        self.reset_srv = self.create_service(Empty, 'reset_odom', self.reset_cb)

        # ── State ─────────────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.last_left    = None
        self.last_right   = None
        self.last_time_ms = None

        # Pending odometry message built in serial thread, published in ROS thread
        self._pending_odom = None
        self._pending_tf   = None
        self._lock = threading.Lock()

        # ── Publish timer (10ms = 100Hz) — runs in ROS executor ──────────
        self.create_timer(0.01, self._publish_pending)

        # ── Serial read thread ────────────────────────────────────────────
        t = threading.Thread(target=self._read_loop, daemon=True)
        t.start()

        self.get_logger().info('serial_odom_node started.')

    # ── Reset service ─────────────────────────────────────────────────────────
    def reset_cb(self, request, response):
        with self._lock:
            self.x = 0.0; self.y = 0.0; self.theta = 0.0
            self.last_left = None; self.last_right = None; self.last_time_ms = None
        try:
            self.ser.write(b'R\n')   # tell Arduino to zero its tick counters too
        except Exception:
            pass
        self.get_logger().info('Odometry reset.')
        return response

    # ── cmd_vel → PWM ─────────────────────────────────────────────────────────
    def cmd_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        vl = v - (TURNING_GAIN * w * WHEEL_BASE / 2.0)
        vr = v + (TURNING_GAIN * w * WHEEL_BASE / 2.0)

        pL = int((vl / MAX_WHEEL_SPEED) * MAX_PWM)
        pR = int((vr / MAX_WHEEL_SPEED) * MAX_PWM)
        pL = max(-MAX_PWM, min(MAX_PWM, pL))
        pR = max(-MAX_PWM, min(MAX_PWM, pR))

        try:
            self.ser.write(f'P,{pL},{pR}\n'.encode())
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')

    # ── Serial read loop (background thread) ──────────────────────────────────
    def _read_loop(self):
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
                    l    = int(parts[1])
                    r    = int(parts[2])
                    t_ms = int(parts[3])
                except ValueError:
                    continue

                # Capture ROS time immediately — before any processing delay
                now_ros = self.get_clock().now()
                self._process_encoders(l, r, t_ms, now_ros)

    # ── Odometry math ─────────────────────────────────────────────────────────
    def _process_encoders(self, l, r, t_ms, now_ros):
        if TICKS_PER_REV == 0:
            return   # not calibrated yet

        with self._lock:
            if self.last_left is None:
                self.last_left = l; self.last_right = r; self.last_time_ms = t_ms
                return

            dl = l - self.last_left
            dr = r - self.last_right

            # Use Arduino dt for kinematics — it's a hardware clock, more accurate
            # than inferring dt from when Linux decides to schedule the read
            dt = (t_ms - self.last_time_ms) / 1000.0

            # Reject bad packets: negative time, stale packets, or suspiciously large jumps
            if dt <= 0.0 or dt > 0.5:
                self.last_left = l; self.last_right = r; self.last_time_ms = t_ms
                return

            # Reject implausible tick deltas - same idea as the dt check above,
            # but for tick counts. A corrupted serial packet can still parse
            # as valid ints while being physically nonsense (e.g. a flipped
            # digit turning "12" into "12000"). Without this, a single bad
            # packet gets permanently integrated into (x, y) as an instant
            # teleport that never goes away until /reset_odom is called.
            max_ticks_this_update = _MAX_PLAUSIBLE_TICKS_PER_SEC * dt
            if abs(dl) > max_ticks_this_update or abs(dr) > max_ticks_this_update:
                self.get_logger().warn(
                    f'Rejected implausible encoder delta: dl={dl}, dr={dr} '
                    f'over dt={dt:.3f}s (max plausible ~{max_ticks_this_update:.0f} '
                    f'ticks) - likely corrupted serial packet, skipping this update.'
                )
                self.last_left = l; self.last_right = r; self.last_time_ms = t_ms
                return

            # Distance each wheel travelled
            dist_l = (dl / TICKS_PER_REV) * (2.0 * math.pi * WHEEL_RADIUS)
            dist_r = (dr / TICKS_PER_REV) * (2.0 * math.pi * WHEEL_RADIUS)

            # Robot motion
            ds     = (dist_r + dist_l) / 2.0
            dtheta = (dist_r - dist_l) / WHEEL_BASE

            # Integrate pose (midpoint method — more accurate than simple Euler)
            self.x     += ds * math.cos(self.theta + dtheta / 2.0)
            self.y     += ds * math.sin(self.theta + dtheta / 2.0)
            self.theta += dtheta

            # Velocities
            vx  = ds     / dt
            vth = dtheta / dt

            # Update state
            self.last_left    = l
            self.last_right   = r
            self.last_time_ms = t_ms

            # Build messages (stamped with ROS time captured at packet arrival)
            stamp = now_ros.to_msg()
            q     = yaw_to_quat(self.theta)

            odom = Odometry()
            odom.header.stamp    = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id  = 'base_link'
            odom.pose.pose.position.x    = self.x
            odom.pose.pose.position.y    = self.y
            odom.pose.pose.position.z    = 0.0
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]

            # Covariance — diagonal elements only (x, y, z, roll, pitch, yaw)
            odom.pose.covariance[0]  = POSE_COVAR_XY   # x
            odom.pose.covariance[7]  = POSE_COVAR_XY   # y
            odom.pose.covariance[14] = 1e6              # z  (ground robot, ignore)
            odom.pose.covariance[21] = 1e6              # roll
            odom.pose.covariance[28] = 1e6              # pitch
            odom.pose.covariance[35] = POSE_COVAR_YAW  # yaw

            odom.twist.twist.linear.x  = vx
            odom.twist.twist.angular.z = vth
            odom.twist.covariance[0]   = TWIST_COVAR_VX
            odom.twist.covariance[35]  = TWIST_COVAR_VYAW

            # TF transform
            tf = TransformStamped()
            tf.header.stamp            = stamp
            tf.header.frame_id         = 'odom'
            tf.child_frame_id          = 'base_link'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x    = q[0]
            tf.transform.rotation.y    = q[1]
            tf.transform.rotation.z    = q[2]
            tf.transform.rotation.w    = q[3]

            # Hand to main thread for publishing
            self._pending_odom = odom
            self._pending_tf   = tf

    # ── Publish in ROS executor thread (thread-safe) ──────────────────────────
    def _publish_pending(self):
        with self._lock:
            odom = self._pending_odom
            tf   = self._pending_tf
            self._pending_odom = None
            self._pending_tf   = None

        if odom is not None:
            self.odom_pub.publish(odom)
            self.tf_brd.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = SerialOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
