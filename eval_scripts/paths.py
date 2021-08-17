import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def gen_t(pts1, pts2):
    tpts = [0]
    for i, pt in enumerate(pts1):
        if i != 0:
            dist_tmp = (pts1[i] - pts1[i-1]) ** 2 + (pts2[i] - pts2[i-1]) ** 2
            tpts += [np.sqrt(dist_tmp) + tpts[-1]]
    maxt = tpts[-1]
    tpts = [t/maxt for t in tpts]
    return tpts

def get_curve(curve, prev=None):
    xpts, ypts = curve['xpts'], curve['ypts']
    order = curve['order']

    if prev is not None:
        init_ts = prev
    else:
        init_ts = curve['init_ts']
    
    xs, ys = xpts[0], ypts[0]
    xf, yf = xpts[-1], ypts[-1]

    tpts = gen_t(xpts, ypts)
    xpoly = np.polynomial.polynomial.Polynomial.fit(tpts, xpts, order)
    ypoly = np.polynomial.polynomial.Polynomial.fit(tpts, ypts, order)
    cx = list(xpoly)[::-1]
    cy = list(ypoly)[::-1]
    
    return xs, ys, xf, yf, init_ts, xpts, ypts, tpts, xpoly, ypoly, cx, cy, order

plt.style.use('ggplot')

T = 10. # Time horizon
N = 40  # number of control intervals
tgrid = [T/N*k for k in range(N+1)]

curr_path = os.path.dirname(__file__) # sorta hacky
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
legend = []

direc = os.path.join(curr_path, 'mpcc_path')
is_mpcc = True

for filename in os.listdir(direc):
    f = os.path.join(direc, filename)
    if os.path.isfile(f):
        csv_tmp = pd.read_csv(f)
        x = list(csv_tmp['x'])
        y = list(csv_tmp['y'])
        alpha = list(csv_tmp['alpha'])
        a = list(csv_tmp['a'])

        lgd_tmp = filename.split('_')
        library = lgd_tmp[0]

        if library == 'CasADi':
            color = 'tab:blue'

            method = lgd_tmp[1]
            solver = 'DMS' if 'rk4' in filename else 'DC'
            label_prefix = library + ' ' + method + ' ' + solver
        else:
            color = 'tab:red'

            method = lgd_tmp[1]
            label_prefix = library + ' ' + method.split('.')[0]

        ax1.plot(range(len(alpha)), alpha, '-', color=color, alpha=0.8, label=label_prefix + r' ($\alpha$)')
        ax1.plot(range(len(a)), a, '--', color=color, alpha=0.8, label=label_prefix + r' ($a$)')

        ax2.plot(x, y, color=color, alpha=0.8, label=label_prefix)

if is_mpcc:
    curve = {'init_ts': np.array([0, 0, np.pi/3, 0, 0, 0]),
                'xpts': [0, .5, 2, 3.3],
                'ypts': [0, 1, 3, 2],
                'order': 5}

    xs, ys, xf, yf, init_ts, xpts, ypts, tpts, xpoly, ypoly, cx, cy, order = get_curve(curve)

    tplt = np.linspace(0, 1)
    xplt = xpoly(tplt)
    yplt = ypoly(tplt)
    ax2.plot(xplt, yplt, '-.', color='grey')
else:
    xs, ys = (0, 0)
    xf, yf = (2, 3)

# ax1.set_title('MPC Control Inputs')
ax1.set_ylabel('Control inputs')
ax1.set_xlabel('Time Horizon')

ax1.set_ylim([-5, 5])

ax2.plot([xs], [ys], marker='.', color='black')
ax2.plot([xf], [yf], marker='x', color='black', markersize=5)

ax2.set_ylim([-5, 5])
ax2.set_xlim([-5, 5])

ax1.legend()
ax2.legend()

name_mp = 'mpcc' if is_mpcc else 'mpc'
plt.savefig(os.path.join(curr_path, '{}_path_comp.png'.format(name_mp)), dpi=300)