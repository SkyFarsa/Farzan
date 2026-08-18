#!/usr/bin/env python3
"""ICM-20948 Hardware Test"""

import time
import sys

print("=" * 60)
print("ICM-20948 HARDWARE TEST")
print("=" * 60)

print("\n[1/3] Checking I2C...")
import os
if os.path.exists('/dev/i2c-1'):
    print("✓ I2C found")
else:
    print("✗ I2C not found - enable with: sudo raspi-config")
    sys.exit(1)

print("\n[2/3] Checking libraries...")
try:
    import smbus2
    print("✓ smbus2 installed")
except ImportError:
    print("✗ Install: pip3 install smbus2")
    sys.exit(1)

try:
    from icm20948 import ICM20948
    print("✓ icm20948 installed")
    lib_ok = True
except ImportError:
    print("⚠ Install: pip3 install icm20948")
    lib_ok = False

print("\n[3/3] Testing sensor...")
if lib_ok:
    try:
        sensor = ICM20948(i2c_addr=0x69)
        time.sleep(0.1)
        
        print("\nReading 5 samples...")
        for i in range(5):
            ax, ay, az, gx, gy, gz = sensor.read_accelerometer_gyro_data()
            print(f"  {i+1}: Accel({ax:.2f}, {ay:.2f}, {az:.2f}g) Gyro({gx:.2f}, {gy:.2f}, {gz:.2f}°/s)")
            time.sleep(0.1)
        
        print("\n✓ Hardware test PASSED!")
        print("\nNext: ros2 run icm20948_ros2_driver calibrate_imu.py")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        print("Check wiring and I2C address (try 0x68)")
        sys.exit(1)

print("\n" + "=" * 60)
