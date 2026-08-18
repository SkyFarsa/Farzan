# motor_control.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.subscriber = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)

    def callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        # Convert to wheel speeds (differential drive)
        left = int((linear - angular * 0.16) * 255)  # Adjust 0.16 to your track width
        right = int((linear + angular * 0.16) * 255)
        command = f"M1:{left},M2:{right}\n"
        self.ser.write(command.encode())

def main():
    rclpy.init()
    node = MotorControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
