#!/usr/bin/env python3
"""ICM-20948 Calibration Script"""

import time
import numpy as np

try:
    from icm20948 import ICM20948
except ImportError:
    print("ERROR: icm20948 library not found!")
    print("Install with: pip3 install icm20948")
    exit(1)

def calibrate_gyroscope(sensor, samples=1000):
    print("\n=== GYROSCOPE CALIBRATION ===")
    print("Keep IMU still...")
    print(f"Collecting {samples} samples...")
    time.sleep(2)
    
    gyro_readings = []
    for i in range(samples):
        _, _, _, gx, gy, gz = sensor.read_accelerometer_gyro_data()
        gyro_readings.append([gx, gy, gz])
        if i % 100 == 0:
            print(f"Progress: {i}/{samples}")
        time.sleep(0.01)
    
    offsets = np.mean(gyro_readings, axis=0)
    print(f"\nGyro offsets: X={offsets[0]:.4f}, Y={offsets[1]:.4f}, Z={offsets[2]:.4f}")
    return offsets

def calibrate_accelerometer(sensor, samples=1000):
    print("\n=== ACCELEROMETER CALIBRATION ===")
    print("Place IMU flat with Z-axis up...")
    print(f"Collecting {samples} samples...")
    time.sleep(2)
    
    accel_readings = []
    for i in range(samples):
        ax, ay, az, _, _, _ = sensor.read_accelerometer_gyro_data()
        accel_readings.append([ax, ay, az])
        if i % 100 == 0:
            print(f"Progress: {i}/{samples}")
        time.sleep(0.01)
    
    mean = np.mean(accel_readings, axis=0)
    offsets = [mean[0], mean[1], mean[2] - 1.0]
    print(f"\nAccel offsets: X={offsets[0]:.4f}, Y={offsets[1]:.4f}, Z={offsets[2]:.4f}")
    return offsets

def main():
    print("=" * 50)
    print("ICM-20948 CALIBRATION TOOL")
    print("=" * 50)
    
    try:
        sensor = ICM20948(i2c_addr=0x69)
        time.sleep(0.1)
        print("Sensor initialized!")
    except Exception as e:
        print(f"ERROR: {e}")
        print("Try address 0x68 or check connections")
        exit(1)
    
    gyro_offsets = calibrate_gyroscope(sensor)
    accel_offsets = calibrate_accelerometer(sensor)
    
    print("\n" + "=" * 50)
    print("Add these to config/icm20948_params.yaml:")
    print("=" * 50)
    print(f"\ngyro_offset_x: {gyro_offsets[0]:.6f}")
    print(f"gyro_offset_y: {gyro_offsets[1]:.6f}")
    print(f"gyro_offset_z: {gyro_offsets[2]:.6f}")
    print(f"\naccel_offset_x: {accel_offsets[0]:.6f}")
    print(f"accel_offset_y: {accel_offsets[1]:.6f}")
    print(f"accel_offset_z: {accel_offsets[2]:.6f}")
    print("\n" + "=" * 50)

if __name__ == '__main__':
    main()
