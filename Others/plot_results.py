#!/usr/bin/env python3
"""plot_results.py <run_folder> : builds plots.html (embedded PNGs) from robot_test_app output."""
import sys, os, json, csv, io, base64
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

def load(p):
    with open(p) as f: r = list(csv.DictReader(f))
    return {k: [float(x[k]) for x in r] for k in r[0]} if r else None

def png(fig):
    b = io.BytesIO(); fig.savefig(b, format='png', dpi=110, bbox_inches='tight'); plt.close(fig)
    return base64.b64encode(b.getvalue()).decode()

def main(d):
    res = json.load(open(os.path.join(d, 'results.json')))['results']
    parts = []
    for r in res:
        d_ = load(os.path.join(d, r['csv'])) if r.get('csv') else None
        if not d_: continue
        fig, ax = plt.subplots(1, 3, figsize=(14, 4)); n = r['test']
        ax[0].plot(d_['x'], d_['y']); ax[0].plot(d_['x'][0], d_['y'][0], 'go'); ax[0].plot(d_['x'][-1], d_['y'][-1], 'rx')
        ax[0].set_aspect('equal'); ax[0].set_title(f'{n}: odom path'); ax[0].set_xlabel('x m'); ax[0].set_ylabel('y m')
        ax[1].plot(d_['t_s'], d_['yaw_rad']); ax[1].set_title('yaw (unwrapped, rad)'); ax[1].set_xlabel('s')
        ax[2].plot(d_['t_s'], d_['vx'], label='vx m/s'); ax[2].plot(d_['t_s'], d_['wz'], label='wz rad/s')
        ax[2].legend(); ax[2].set_title('velocities'); ax[2].set_xlabel('s')
        info = '<br>'.join(f'{k}: {round(v,3) if isinstance(v,float) else v}' for k, v in r.items() if k != 'test')
        parts.append(f'<h2>{n}</h2><img style="max-width:100%" src="data:image/png;base64,{png(fig)}"><p>{info}</p>')
    # summary: error per test type
    sq = [r for r in res if r['test'].startswith('square')]
    if sq:
        fig, ax = plt.subplots(figsize=(4.5, 4.5))
        for r in sq: ax.scatter(r['real_err_x'], r['real_err_y'], label=r['test'])
        ax.axhline(0, c='gray'); ax.axvline(0, c='gray'); ax.set_aspect('equal'); ax.legend()
        ax.set_title('Square return error (m)'); parts.append(f'<h2>UMBmark scatter</h2><img src="data:image/png;base64,{png(fig)}">')
    html = '<!DOCTYPE html><meta charset="utf-8"><title>Plots</title><body style="font:15px system-ui;max-width:1000px;margin:2em auto">' + '<h1>Test plots</h1>' + ''.join(parts)
    out = os.path.join(d, 'plots.html'); open(out, 'w').write(html); print('Wrote', out)

if __name__ == '__main__': main(sys.argv[1])
