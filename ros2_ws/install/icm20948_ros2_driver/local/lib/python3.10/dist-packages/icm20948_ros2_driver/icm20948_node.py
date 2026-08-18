#!/usr/bin/env python3
"""
ICM-20948 ROS2 Driver Node
Publishes IMU data (accelerometer, gyroscope, magnetometer) to ROS2 topics
Supports both I2C and SPI connections
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Header
import numpy as np
import time

try:
    from icm20948 import ICM20948
    ICM20948_AVAILABLE = True
except ImportError:
    ICM20948_AVAILABLE = False
    print("Warning: icm20948 library not found. Install with: pip3 install icm20948")


class ICM20948Node(Node):
    """ROS2 Node for ICM-20948 9-axis IMU sensor"""
    
    def __init__(self):
        super().__init__('icm20948_node')
        
        # Declare parameters
        self.declare_parameter('connection_type', 'i2c')
        self.declare_parameter('i2c_address', 0x69)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('publish_mag', True)
        self.declare_parameter('publish_temp', True)
        self.declare_parameter('gyro_offset_x', 0.0)
        self.declare_parameter('gyro_offset_y', 0.0)
        self.declare_parameter('gyro_offset_z', 0.0)
        self.declare_parameter('accel_offset_x', 0.0)
        self.declare_parameter('accel_offset_y', 0.0)
        self.declare_parameter('accel_offset_z', 0.0)
        
        # Get parameters
        self.connection_type = self.get_parameter('connection_type').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_mag = self.get_parameter('publish_mag').value
        self.publish_temp = self.get_parameter('publish_temp').value
        
        # Get calibration offsets
        self.gyro_offset = np.array([
            self.get_parameter('gyro_offset_x').value,
            self.get_parameter('gyro_offset_y').value,
            self.get_parameter('gyro_offset_z').value
        ])
        self.accel_offset = np.array([
            self.get_parameter('accel_offset_x').value,
            self.get_parameter('accel_offset_y').value,
            self.get_parameter('accel_offset_z').value
        ])
        
        # Initialize sensor
        self.sensor = None
        self.initialize_sensor()
        
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        
        if self.publish_mag:
            self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
            
        if self.publish_temp:
            self.temp_pub = self.create_publisher(Temperature, '/imu/temperature', 10)
        
        # Create timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'ICM-20948 node started - Publishing at {self.publish_rate} Hz')
        
    def initialize_sensor(self):
        """Initialize the ICM-20948 sensor"""
        if not ICM20948_AVAILABLE:
            self.get_logger().error('ICM20948 library not available!')
            return
            
        try:
            if self.connection_type == 'i2c':
                self.get_logger().info(f'Initializing on I2C bus {self.i2c_bus}, address 0x{self.i2c_address:02X}')
                self.sensor = ICM20948(i2c_addr=self.i2c_address, i2c_bus=self.i2c_bus)
            else:
                self.get_logger().error(f'Invalid connection type: {self.connection_type}')
                return
                
            time.sleep(0.1)
            ax, ay, az, gx, gy, gz = self.sensor.read_accelerometer_gyro_data()
            self.get_logger().info(f'Sensor initialized! Test: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize sensor: {str(e)}')
            self.sensor = None
    
    def timer_callback(self):
        """Publish IMU data"""
        if self.sensor is None:
            return
            
        try:
            ax, ay, az, gx, gy, gz = self.sensor.read_accelerometer_gyro_data()
            
            gx -= self.gyro_offset[0]
            gy -= self.gyro_offset[1]
            gz -= self.gyro_offset[2]
            ax -= self.accel_offset[0]
            ay -= self.accel_offset[1]
            az -= self.accel_offset[2]
            
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            imu_msg.linear_acceleration.x = float(ax * 9.80665)
            imu_msg.linear_acceleration.y = float(ay * 9.80665)
            imu_msg.linear_acceleration.z = float(az * 9.80665)
            
            imu_msg.angular_velocity.x = float(np.radians(gx))
            imu_msg.angular_velocity.y = float(np.radians(gy))
            imu_msg.angular_velocity.z = float(np.radians(gz))
            
            imu_msg.orientation.w = 1.0
            
            imu_msg.orientation_covariance = [-1.0] + [0.0] * 8
            
            gyro_cov = 0.01 ** 2
            imu_msg.angular_velocity_covariance = [
                gyro_cov, 0.0, 0.0,
                0.0, gyro_cov, 0.0,
                0.0, 0.0, gyro_cov
            ]
            
            accel_cov = 0.1 ** 2
            imu_msg.linear_acceleration_covariance = [
                accel_cov, 0.0, 0.0,
                0.0, accel_cov, 0.0,
                0.0, 0.0, accel_cov
            ]
            
            self.imu_pub.publish(imu_msg)
            
            if self.publish_mag:
                try:
                    mx, my, mz = self.sensor.read_magnetometer_data()
                    mag_msg = MagneticField()
                    mag_msg.header = imu_msg.header
                    mag_msg.magnetic_field.x = float(mx * 1e-6)
                    mag_msg.magnetic_field.y = float(my * 1e-6)
                    mag_msg.magnetic_field.z = float(mz * 1e-6)
                    mag_cov = (0.1e-6) ** 2
                    mag_msg.magnetic_field_covariance = [
                        mag_cov, 0.0, 0.0,
                        0.0, mag_cov, 0.0,
                        0.0, 0.0, mag_cov
                    ]
                    self.mag_pub.publish(mag_msg)
                except:
                    pass
            
            if self.publish_temp:
                try:
                    temp = self.sensor.read_temperature()
                    temp_msg = Temperature()
                    temp_msg.header = imu_msg.header
                    temp_msg.temperature = float(temp)
                    temp_msg.variance = 1.0
                    self.temp_pub.publish(temp_msg)
                except:
                    pass
                    
        except Exception as e:
            self.get_logger().error(f'Error reading sensor: {str(e)}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ICM20948Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
