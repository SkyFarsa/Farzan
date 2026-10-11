#!/usr/bin/env python3
"""
find_min_pwm.py - finds the real stall threshold for your direct-drive
motors, objectively, using encoder tick deltas instead of listening for
whether the motor "sounds" like it's turning.

IMPORTANT: stop robot.launch.py first! serial_odom_node.py holds the
serial port open, and this script needs exclusive access to it too.

Usage:
    python3 find_min_pwm.py

Tests both wheels together at increasing PWM levels (starting low, well
below where you'd expect stall), reads real encoder ticks before/after
each step, and reports the first level where BOTH wheels show a real,
consistent tick count increase - that's your actual minimum effective PWM.
Always stops the motors safely on exit, including on Ctrl+C or an error.
"""

import serial
import time
import sys

PORT = '/dev/turtlebot_arduino'   # change if needed (or use /dev/ttyACM0)
BAUD = 115200

START_PWM = 20
END_PWM = 150
STEP = 10
HOLD_SECONDS = 1.2   # how long to command each PWM level before checking ticks


def read_latest_ticks(ser, timeout=0.3):
    """Read whatever encoder lines have arrived, return the most recent
    (left, right) tick counts, or None if nothing valid was received."""
    deadline = time.time() + timeout
    latest = None
    while time.time() < deadline:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if not line:
            continue
        parts = line.split(',')
        if parts[0] == 'E' and len(parts) >= 4:
            try:
                l = int(parts[1])
                r = int(parts[2])
                latest = (l, r)
            except ValueError:
                continue
    return latest


def send_pwm(ser, pwm_l, pwm_r):
    ser.write(f'P,{pwm_l},{pwm_r}\n'.encode())


def stop_motors(ser):
    send_pwm(ser, 0, 0)
    time.sleep(0.1)
    send_pwm(ser, 0, 0)  # send twice, just in case the first is missed


def main():
    print(f'Opening {PORT} at {BAUD} baud...')
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f'ERROR: could not open {PORT}: {e}')
        print('Is robot.launch.py (or anything else) still running and holding the port?')
        sys.exit(1)

    time.sleep(2.0)  # let the Arduino finish its boot/reset after opening the port
    ser.reset_input_buffer()

    print('\n*** Make sure the robot is up on a stand / wheels off the ground, or in a clear space! ***\n')
    input('Press Enter when ready to start the ramp test...')

    found_threshold = None

    try:
        for pwm in range(START_PWM, END_PWM + 1, STEP):
            print(f'\nTesting PWM = {pwm} ...')

            before = read_latest_ticks(ser, timeout=0.3)
            send_pwm(ser, pwm, pwm)

            after = read_latest_ticks(ser, timeout=HOLD_SECONDS)

            stop_motors(ser)
            time.sleep(0.5)  # let it fully stop before the next step

            if before is None or after is None:
                print('  (no encoder data received - check wiring/port)')
                continue

            dl = after[0] - before[0]
            dr = after[1] - before[1]
            print(f'  left ticks: {dl}   right ticks: {dr}')

            # Require BOTH wheels to show real, non-trivial movement -
            # a single wheel moving could just be one side already past
            # its own threshold while the other is still stalled.
            if abs(dl) >= 3 and abs(dr) >= 3:
                found_threshold = pwm
                print(f'  --> Both wheels moved. This looks like your threshold.')
                break
            else:
                print('  --> Not enough movement yet (or just buzzing), trying higher PWM...')

    except KeyboardInterrupt:
        print('\nInterrupted by user.')
    finally:
        stop_motors(ser)
        ser.close()
        print('\nMotors stopped, serial port closed.')

    if found_threshold is not None:
        recommended = found_threshold + 15  # small safety margin above the raw threshold
        print(f'\n=== RESULT ===')
        print(f'First PWM level with real movement on both wheels: {found_threshold}')
        print(f'Recommended MIN_EFFECTIVE_PWM (with margin): {recommended}')
        print(f'Set this in arduino_firmware.ino:  #define MIN_EFFECTIVE_PWM {recommended}')
    else:
        print('\nNo clear threshold found in the tested range '
              f'({START_PWM}-{END_PWM}). Either the motors need a higher '
              'PWM than tested, or check wiring/encoder connections.')


if __name__ == '__main__':
    main()
