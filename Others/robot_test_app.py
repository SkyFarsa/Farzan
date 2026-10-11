#!/usr/bin/env python3
"""
robot_test_app.py: interactive test + data-logging tool for the triangle_bot.

Run on the machine that sees your ROS 2 topics (RPi4 or remote PC, ROS_DOMAIN_ID=30),
AFTER `ros2 launch my_robot robot.launch.py` is running:

    python3 robot_test_app.py                       # defaults: TPR=203, base=0.48
    python3 robot_test_app.py --tpr 203 --base 0.48 --out ~/robot_tests

Menu tests: 1 stationary health monitor, 2 straight line, 3 spin in place,
            4 square (UMBmark-style), 5 write report + quit.
Each test can be AUTO (app drives via /cmd_vel using odometry) or MANUAL
(you drive with teleop). You then type in the tape-measured result.
Outputs (per session folder): <test>_odom.csv, results.json, report.html
with suggested corrected TICKS_PER_REV / WHEEL_BASE.

SAFETY: keep the robot on the floor in a clear area, e-stop in reach, start with
low speeds. serial_odom_node has no command watchdog: the app sends zero Twists
on finish or Ctrl+C, but if the app dies the wheels may keep their last command.
"""
import argparse, csv, json, math, os, threading, time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


class Rate:
    def __init__(s): s.reset()
    def reset(s): s.n = 0; s.t0 = None; s.last = None; s.maxgap = 0.0
    def tick(s):
        t = time.monotonic()
        if s.t0 is None: s.t0 = t
        if s.last is not None: s.maxgap = max(s.maxgap, t - s.last)
        s.last = t; s.n += 1
    def hz(s):
        return (s.n - 1) / (s.last - s.t0) if s.n > 1 and s.last > s.t0 else 0.0


class TestNode(Node):
    def __init__(self):
        super().__init__('robot_test_app')
        self.rates = {k: Rate() for k in ('odom', 'scan', 'imu')}
        self.x = self.y = self.yaw = 0.0          # yaw is unwrapped (continuous)
        self._raw = None; self.have_odom = False
        self.rec = None; self.t_rec = 0.0
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.create_subscription(LaserScan, '/scan', lambda m: self.rates['scan'].tick(), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', lambda m: self.rates['imu'].tick(), qos_profile_sensor_data)
        self.reset_cli = self.create_client(Empty, '/reset_odom')

    def _odom(self, m):
        self.rates['odom'].tick()
        q = m.pose.pose.orientation
        raw = 2.0 * math.atan2(q.z, q.w)
        if self._raw is None: self.yaw = raw
        else: self.yaw += (raw - self._raw + math.pi) % (2 * math.pi) - math.pi
        self._raw = raw
        self.x, self.y = m.pose.pose.position.x, m.pose.pose.position.y
        self.have_odom = True
        if self.rec is not None:
            self.rec.append((round(time.time() - self.t_rec, 3), self.x, self.y, self.yaw,
                             m.twist.twist.linear.x, m.twist.twist.angular.z))

    def send(self, v, w):
        t = Twist(); t.linear.x = float(v); t.angular.z = float(w); self.pub.publish(t)

    def pose(self): return (self.x, self.y, self.yaw)

    def reset_odom(self):
        if self.reset_cli.wait_for_service(timeout_sec=1.0):
            self.reset_cli.call_async(Empty.Request()); time.sleep(0.6)
            return True
        print('  (reset_odom service not found; using start-pose snapshot instead)')
        return False

    def drive(self, v, w, done, timeout):
        t0 = time.time()
        try:
            while time.time() - t0 < timeout and not done():
                self.send(v, w); time.sleep(0.05)
        finally:
            for _ in range(6): self.send(0, 0); time.sleep(0.05)
        return time.time() - t0


# ---------------------------------------------------------------- helpers
def ask(prompt, default=None, cast=float):
    while True:
        s = input(f'{prompt}' + (f' [{default}]' if default is not None else '') + ': ').strip()
        if not s and default is not None: return default
        try: return cast(s)
        except ValueError: print('  invalid input')

def save_csv(node, outdir, name, rows):
    p = os.path.join(outdir, f'{name}_odom.csv')
    with open(p, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['t_s', 'x', 'y', 'yaw_rad', 'vx', 'wz']); w.writerows(rows)
    return os.path.basename(p)

def begin(node, auto_msg):
    node.reset_odom(); time.sleep(0.3)
    p0 = node.pose(); node.t_rec = time.time(); node.rec = []
    return p0

def finish(node):
    rows, node.rec = node.rec, None
    return rows


# ---------------------------------------------------------------- tests
def t_monitor(node, a, out):
    secs = ask('Duration seconds (wheels stationary)', 30, int)
    for r in node.rates.values(): r.reset()
    l0 = os.getloadavg()[0]; p0 = node.pose(); node.t_rec = time.time(); node.rec = []
    for i in range(secs): time.sleep(1); print(f'\r  {i+1}/{secs}s', end='', flush=True)
    print(); p1 = node.pose(); rows = finish(node)
    res = {'test': 'monitor', 'seconds': secs, 'load_start': l0, 'load_end': os.getloadavg()[0],
           'odom_drift_m': math.hypot(p1[0]-p0[0], p1[1]-p0[1]), 'yaw_drift_deg': math.degrees(p1[2]-p0[2]),
           'csv': save_csv(node, out, 'monitor', rows)}
    for k, r in node.rates.items():
        res[f'{k}_hz'] = round(r.hz(), 2); res[f'{k}_max_gap_s'] = round(r.maxgap, 3)
    print('  ' + json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in res.items()}))
    return res

def t_straight(node, a, out):
    auto = ask('Mode: 1=auto 2=manual', 1, int) == 1
    target = ask('Target distance m', 2.0); p0 = begin(node, '')
    if auto:
        v = ask('Speed m/s (keep low)', 0.15)
        input('  Clear path ahead? Enter to GO (Ctrl+C = stop)')
        node.drive(v, 0, lambda: math.hypot(node.x-p0[0], node.y-p0[1]) >= target, target/v*2+5)
    else:
        input('  Drive the robot yourself, then press Enter when stopped')
    time.sleep(0.5); p1 = node.pose(); rows = finish(node)
    od = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
    actual = ask('Tape-measured distance actually travelled (m)', target)
    scale = od / actual if actual else 1.0
    res = {'test': 'straight', 'odom_dist_m': od, 'actual_dist_m': actual, 'error_pct': (od-actual)/actual*100,
           'heading_change_deg': math.degrees(p1[2]-p0[2]), 'suggested_ticks_per_rev': a.tpr * scale,
           'csv': save_csv(node, out, 'straight', rows)}
    print(f"  odom {od:.3f} m vs actual {actual:.3f} m ({res['error_pct']:+.2f}%) -> TICKS_PER_REV ~ {res['suggested_ticks_per_rev']:.1f}")
    return res

def t_spin(node, a, out):
    auto = ask('Mode: 1=auto 2=manual', 1, int) == 1
    n = ask('Full rotations', 3.0); p0 = begin(node, '')
    if auto:
        w = ask('Angular speed rad/s', 0.6); d = ask('Direction +1=CCW -1=CW', 1, int)
        input('  Mark the robot heading on the floor. Enter to GO')
        node.drive(0, w*d, lambda: abs(node.yaw-p0[2]) >= n*2*math.pi, n*2*math.pi/w*2+5)
    else:
        input('  Rotate the robot in place yourself, Enter when stopped')
    time.sleep(0.5); p1 = node.pose(); rows = finish(node)
    est = abs(math.degrees(p1[2]-p0[2]))
    print(f'  odometry says {est:.1f} deg turned')
    actual = ask('Actual total rotation in degrees (e.g. 1080 or 1095)', round(est))
    base_new = a.base * est / actual if actual else a.base
    res = {'test': 'spin', 'odom_deg': est, 'actual_deg': actual, 'error_deg': est-actual,
           'suggested_wheel_base': base_new, 'csv': save_csv(node, out, 'spin', rows)}
    print(f'  error {est-actual:+.1f} deg -> WHEEL_BASE ~ {base_new:.4f} m')
    return res

def t_square(node, a, out):
    auto = ask('Mode: 1=auto 2=manual', 1, int) == 1
    side = ask('Side length m', 1.0); d = ask('Direction +1=CCW -1=CW', 1, int); p0 = begin(node, '')
    if auto:
        v = ask('Speed m/s', 0.15); w = 0.5
        input('  Mark start position and heading. Enter to GO')
        for i in range(4):
            ps = node.pose()
            node.drive(v, 0, lambda: math.hypot(node.x-ps[0], node.y-ps[1]) >= side, side/v*2+5)
            time.sleep(0.4); ys = node.yaw
            node.drive(0, w*d, lambda: abs(node.yaw-ys) >= math.pi/2 - 0.03, 15)
            time.sleep(0.4); print(f'  side {i+1}/4 done')
    else:
        input('  Drive the square yourself, return to start, Enter when stopped')
    time.sleep(0.5); p1 = node.pose(); rows = finish(node)
    print(f'  odom end pose: x={p1[0]:.3f} y={p1[1]:.3f} yaw={math.degrees(p1[2]):.1f} deg')
    ex = ask('Measured end offset FORWARD of start mark (m, negative=behind)', 0.0)
    ey = ask('Measured end offset LEFT of start mark (m, negative=right)', 0.0)
    eh = ask('Measured heading error (deg, CCW positive)', 0.0)
    res = {'test': 'square_' + ('ccw' if d > 0 else 'cw'), 'side_m': side, 'odom_end_x': p1[0], 'odom_end_y': p1[1],
           'odom_end_yaw_deg': math.degrees(p1[2]), 'real_err_x': ex, 'real_err_y': ey,
           'real_err_m': math.hypot(ex, ey), 'real_heading_err_deg': eh,
           'csv': save_csv(node, out, 'square_' + ('ccw' if d > 0 else 'cw'), rows)}
    print(f"  real return error {res['real_err_m']*100:.1f} cm, {eh:+.1f} deg")
    return res


# ---------------------------------------------------------------- report
def write_report(out, results, a):
    last = lambda t, k: next((r[k] for r in reversed(results) if r['test'] == t and k in r), None)
    sug_t, sug_b = last('straight', 'suggested_ticks_per_rev'), last('spin', 'suggested_wheel_base')
    rows = ''.join('<tr><td>{}</td><td>{}</td></tr>'.format(r['test'], '<br>'.join(
        f'{k}: {round(v, 3) if isinstance(v, float) else v}' for k, v in r.items() if k != 'test')) for r in results)
    html = f'''<!DOCTYPE html><meta charset="utf-8"><title>Robot test report</title>
<style>body{{font:15px system-ui;max-width:800px;margin:2em auto;padding:0 1em}}td,th{{border:1px solid #ccc;padding:6px;vertical-align:top}}table{{border-collapse:collapse;width:100%}}</style>
<h1>Robot test report</h1><p>{datetime.now():%Y-%m-%d %H:%M} &middot; current TICKS_PER_REV={a.tpr}, WHEEL_BASE={a.base}</p>
<h2>Suggested constants</h2><ul><li>TICKS_PER_REV: {f'{sug_t:.1f}' if sug_t else 'run straight test'}</li>
<li>WHEEL_BASE: {f'{sug_b:.4f}' if sug_b else 'run spin test'}</li></ul>
<p>Re-run the tests after changing constants to confirm. Suggestions from single runs are noisy; average several.</p>
<h2>Results</h2><table><tr><th>Test</th><th>Data</th></tr>{rows}</table>'''
    open(os.path.join(out, 'report.html'), 'w').write(html)
    json.dump({'params': vars(a), 'results': results}, open(os.path.join(out, 'results.json'), 'w'), indent=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tpr', type=float, default=203); ap.add_argument('--base', type=float, default=0.48)
    ap.add_argument('--out', default=os.path.expanduser('~/robot_tests'))
    a = ap.parse_args()
    out = os.path.join(a.out, datetime.now().strftime('run_%Y%m%d_%H%M%S')); os.makedirs(out, exist_ok=True)
    rclpy.init(); node = TestNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print(f'Saving to {out}\nWaiting for /odom ...')
    while not node.have_odom: time.sleep(0.2)
    results = []
    tests = {'1': t_monitor, '2': t_straight, '3': t_spin, '4': t_square}
    try:
        while True:
            c = input('\n1 monitor | 2 straight | 3 spin | 4 square | 5 report+quit > ').strip()
            if c == '5': break
            if c in tests:
                try:
                    results.append(tests[c](node, a, out)); write_report(out, results, a)
                except KeyboardInterrupt:
                    node.send(0, 0); print('\n  stopped')
    finally:
        for _ in range(5): node.send(0, 0); time.sleep(0.05)
        write_report(out, results, a); print(f'Report: {out}/report.html')
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
