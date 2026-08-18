# encoder_reader.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_reader')
        self.publisher = self.create_publisher(Int32MultiArray, '/encoder_data', 10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_encoders)

    def read_encoders(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode().strip()
            if line.startswith('E1:'):
                data = list(map(int, line.split(':')[1].split(',')))
                msg = Int32MultiArray(data=data)
                self.publisher.publish(msg)

def main():
    rclpy.init()
    node = EncoderReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
