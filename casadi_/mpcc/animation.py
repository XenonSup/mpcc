from casadi_.mpcc.utils import get_curve, compute_step, get_timing
from casadi_.mpcc.loss import gen_cost_func
from casadi_.mpcc.bounds import BoundAndGuess

import casadi as cd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import csv
import time
import casadi_.mpcc.config as cfg

from casadi_.solvers.mpcc_rk4 import build_solver as solver_rk4
from casadi_.solvers.mpcc_colloc import build_solver as solver_colloc

plt.style.use('ggplot')

# Logging: Prep
if cfg.log_simple_time:
    simple_time_csv = open(cfg.simple_time_csv, 'w')
    simple_time_writer = csv.writer(simple_time_csv)

if cfg.log_path:
    path_csv = open(cfg.path_csv, 'w')
    path_writer = csv.writer(path_csv)
    path_writer.writerow(['x', 'y', 'alpha', 'a'])

build_solver = solver_rk4 if cfg.solve_method == 'rk4' else solver_colloc

# Problem parameters
T = cfg.T
N = cfg.N
inter_axle = cfg.inter_axle

ts = cfg.ts
e = cfg.e

rebuild_solver = False  # On first curve
keep_going = True       # Target not reached
num_targets = 0

fig, (ax1, ax2) =  plt.subplots(1, 2, figsize=(10, 5))

### Begin Initialization

# Initial path
curve = cfg.curves_lst[0]
xs, ys, xf, yf, init_ts, xpts, ypts, tpts, xpoly, ypoly, cx, cy, order = get_curve(curve)

print(cx, cy)

# Generate cost function for given order. Not being used
# Can supply any curve and any state otherwise
# cost_func = gen_cost_func(order)

# solver, params, trajectories = build_solver(init_ts, T, N, inter_axle, order, xpoly, ypoly)
# w0_suffix, lbw_suffix, ubw_suffix, lbg, ubg = params
solver, trajectories = build_solver(init_ts, T, N, inter_axle, order, xpoly, ypoly)

bounds = BoundAndGuess(init_ts, N, xpoly, ypoly)

w0, lbw, ubw, lbg, ubg = bounds.get_all_bounds()

# Run first instance of solver, get solutions
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=cd.vertcat(cx, cy))

w_opt = sol['x'].full().flatten()
state_opt, u_opt = trajectories(sol['x'])
state_opt = state_opt.full() # to numpy array
u_opt = u_opt.full() # to numpy array

if cfg.log_path:
    path_writer.writerow([state_opt[0][0], state_opt[1][0], u_opt[0][0], u_opt[1][0]])
def solve_mpcc():
    global sol, rebuild_solver, bounds

    if rebuild_solver:
        global solver
        
        # global update? local variables?
        # lbw[:6] = init_ts -> UnboundLocalError: local variable 'lbw' referenced before assignment
        # lbg[:6] = init_ts -> No error?!

        # local variables
        w0, lbw, ubw = bounds.update(init_ts, xpoly, ypoly)

        sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=cd.vertcat(cx, cy))

        print('Rebuilt solver')
        rebuild_solver = False

    else:
        lbw, ubw = bounds.update_wbounds(init_ts)
        t0 = time.time()
        sol = solver(x0=sol['x'], lam_x0=sol['lam_x'], lam_g0=sol['lam_g'], lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=cd.vertcat(cx, cy))
        t1 = time.time()
        if cfg.log_simple_time:
            simple_time_writer.writerow([t1-t0])

    # cost = sol['f'].full().flatten()

    state_opt, u_opt = trajectories(sol['x'])
    state_opt = state_opt.full() # to numpy array
    u_opt = u_opt.full() # to numpy array

    if cfg.log_path:
        path_writer.writerow([state_opt[0][0], state_opt[1][0], u_opt[0][0], u_opt[1][0]])

    return state_opt, u_opt

def gen():
    # Yields a dummy frame number to keep solver animation going
    # and updates curve when completed
    global keep_going, num_targets
    i = 0
    # Keeps yielding as long as there are still targets
    while num_targets < cfg.num_targets_final:

        # If reached intermediate target
        if not keep_going:

            num_targets += 1 # increment target tracker

            # if there still are targets
            if num_targets < cfg.num_targets_final:

                global xs, ys, xf, yf, init_ts, xpts, ypts, tpts, xpoly, ypoly, cx, cy, order, xplt, yplt

                # Update curve
                curve = cfg.curves_lst[num_targets]
                xs, ys, xf, yf, init_ts, xpts, ypts, tpts, xpoly, ypoly, cx, cy, order = get_curve(curve, prev=init_ts)
                print('Updated init_ts')

                # Update curve plot
                tplt = np.linspace(0, 1)
                xplt = xpoly(tplt)
                yplt = ypoly(tplt)
                
                # Used to be in the outer if, after the else
                print('number of targets reached:', num_targets)
                keep_going = True

            # if there are no more targets: Stop iteration
            else: break 


        else: i += 1
        yield i

def update(i):
    global init_ts, keep_going, rebuild_solver

    # Solve the MPCC problem, get state and control vectors
    state_opt, u_opt = solve_mpcc()

    # Get the first state and control vectors from the solution
    # Initial state vector should match the initial conditions
    init = [state_opt[0][0], state_opt[1][0], state_opt[2][0], state_opt[3][0], state_opt[4][0], state_opt[5][0], u_opt[0][0], u_opt[1][0], u_opt[2][0]]

    # Simulate the state forward one step, acquire new initial conditions
    init_ts = compute_step(init, ts, inter_axle)

    # If next state is within epsilon of target, target is reached
    if abs(init_ts[0]-xf) < e and abs(init_ts[1]-yf) < e:
        print('target reached:', xf, yf)
        keep_going = False
        rebuild_solver = True
    
    ## Update plot data
    x_diff = [xf - x for x in state_opt[0]]
    y_diff = [yf - y for y in state_opt[1]]

    x_line.set_ydata(x_diff)
    y_line.set_ydata(y_diff)

    aux_line.set_ydata(np.append(np.nan, u_opt[1]))
    alphaux_line.set_ydata(np.append(np.nan, u_opt[0]))

    traj.set_data(state_opt[0], state_opt[1])
    curr_pt.set_data([state_opt[0][0]], [state_opt[1][0]])
    target_pt.set_data([xf], [yf])

    curve_pts.set_offsets(np.c_[xpts, ypts])
    curve_ln.set_data(xplt, yplt)

    # Animation function stuff
    return [x_line, y_line, aux_line, alphaux_line, traj, curr_pt]

### PLOTTING ###

tgrid = [T/N*k for k in range(N+1)]

# Calculate the error
x_diff = [xf - x for x in state_opt[0]]
y_diff = [yf - y for y in state_opt[1]]

ax1.set_xlim([0, int(T)])
ax1.set_ylim([-5, 5])
x_line, = ax1.plot(tgrid, x_diff, '-', color='gray')
y_line, = ax1.plot(tgrid, y_diff, '-', color='black')
aux_line, = ax1.step(tgrid, np.append(np.nan, u_opt[1]), '-.', color='green')
alphaux_line, = ax1.step(tgrid, np.append(np.nan, u_opt[0]), '-.', color='blue')

# Straight line bounds, should reference a constant by name
amin, amax = -1, 1
alphamin, alphamax = -np.pi, np.pi

amin_pts = [amin for _ in tgrid]
amax_pts = [amax for _ in tgrid]
alphamin_pts = [alphamin for _ in tgrid]
alphamax_pts = [alphamax for _ in tgrid]

ax1.plot(tgrid, amin_pts, '--', color='green', alpha=0.3)
ax1.plot(tgrid, amax_pts, '--', color='green', alpha=0.3)
ax1.plot(tgrid, alphamin_pts, '--', color='blue', alpha=0.3)
ax1.plot(tgrid, alphamax_pts, '--', color='blue', alpha=0.3)

ax1.legend([r'$x_f - x$',r'$y_f - y$', r'$a$ Acceleration', r'$\alpha$ Steering Wheel Vel'])
ax1.set_xlabel('Time horizon')
ax1.grid(True)

ax2.set_ylim([-6, 6])
ax2.set_xlim([-6, 6])
ax2.set_ylabel('y-axis')
ax2.set_xlabel('x-axis')

# plot path polynomial
curve_pts = ax2.scatter(xpts, ypts, color='grey', s=15)
tplt = np.linspace(0, 1)
xplt = xpoly(tplt)
yplt = ypoly(tplt)
curve_ln, = ax2.plot(xplt, yplt, '-.', color='grey')

# plot solution path
traj, = ax2.plot(state_opt[0], state_opt[1], '-', color='green', alpha=0.4)
curr_pt, = ax2.plot([state_opt[0][0]], [state_opt[1][0]], marker='o', color='black')    
target_pt, = ax2.plot([xf], [yf], marker='x', color='black')
ax2.grid(True)

anim = animation.FuncAnimation(fig, update, interval=100, frames=gen, save_count=3000)
# writergif = animation.PillowWriter(fps=30)
# anim.save(cfg.anim_save_file, writer=writergif)
plt.show()

### Logging: Conclude 

if cfg.log_time:
    with open(cfg.out_log_file, 'r') as f:
        tmp = ' '.join(f.read().split('\n'))
    timing = get_timing(tmp)

    time_fl = open(cfg.time_csv, 'w')
    time_writer = csv.writer(time_fl)
    time_writer.writerow(['IN_IPOPT', 'IN_NLP'])
    for t in timing:
        time_writer.writerow(list(t))
    time_fl.close()

if cfg.log_simple_time:
    simple_time_csv.close()

if cfg.log_path:
    path_csv.close()