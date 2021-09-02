from casadi_.mpcc.loss import gen_cost_func
from casadi_.mpcc.utils import merge_dict
from os import system

import os
import casadi as cd
import random as rd
import numpy as np
import matplotlib.pyplot as plt

import casadi_.mpcc.config as cfg

def build_solver(init_ts, T, N, D, order, xpoly, ypoly):
    # State variables
    x = cd.SX.sym('x')
    y = cd.SX.sym('y')
    phi = cd.SX.sym('phi')
    delta = cd.SX.sym('delta')
    vx = cd.SX.sym('vx')
    theta = cd.SX.sym('theta')

    z = cd.vertcat(x, y, phi, delta, vx, theta)

    # Control variables
    alphaux = cd.SX.sym('alphaux')
    aux = cd.SX.sym('aux')
    dt = cd.SX.sym('dt')

    u = cd.vertcat(alphaux, aux, dt)

    # State evolution
    zdot = cd.vertcat(vx*cd.cos(phi), vx*cd.sin(phi), (vx/D)*cd.tan(delta), alphaux, aux, vx*dt)

    # Contours
            #  sym(name,   nrow   , ncol)
    xc = cd.SX.sym('xc', order + 1, 1)
    yc = cd.SX.sym('yc', order + 1, 1)
    contour_cost = gen_cost_func(order)
    # Cost is only a function of order

    L = contour_cost(pos=cd.vertcat(x, y), a=aux, alpha=alphaux, dt=dt, t=theta, t_dest=1.0, cx=xc, cy=yc)['cost']

    # Fixed step Runge-Kutta 4 integrator
    M = 4 # RK4 steps per interval
    DT = T/N/M
    f = cd.Function('f', [z, u], [zdot, L])
    X0 = cd.SX.sym('X0', 6)
    U = cd.SX.sym('U', 3)
    X = X0
    Q = 0
    for j in range(M):
        k1, k1_q = f(X, U)
        k2, k2_q = f(X + DT/2 * k1, U)
        k3, k3_q = f(X + DT/2 * k2, U)
        k4, k4_q = f(X + DT * k3, U)
        X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
        Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)

    F = cd.Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])

    # Start with an empty NLP
    w=[]
    w0 = []
    lbw = []
    ubw = []
    J = 0
    g=[]
    lbg = []
    ubg = []

    # For plotting x and u given w
    coord_plot = []
    u_plot = []

    # "Lift" initial conditions
    Xk = cd.SX.sym('X0', 6)
    w += [Xk]
    lbw += init_ts
    ubw += init_ts
    w0  += init_ts
    coord_plot += [Xk]

    # Formulate the NLP
    for k in range(N):
        # New NLP variable for the control
        Uk = cd.SX.sym('U_' + str(k), 3)
        w   += [Uk]
        #       alphaux  aux  dt
        lbw += [-2*cd.pi, -1, 0]
        ubw += [ 2*cd.pi,  1, 1]
        w0  += [rd.randint(-628, 628)/1000., rd.randint(-100, 100)/1000., rd.randint(0, 100)/1000.]
        u_plot += [Uk]

        # Integrate till the end of the interval
        Fk = F(x0=Xk, p=Uk)
        Xk_end = Fk['xf']
        J=J+Fk['qf']

        kf = float(k)
        theta_tmp = kf/(N-1)
        dtheta = 0.2

        # New NLP variable for state at end of interval
        Xk = cd.SX.sym('X_' + str(k+1), 6)
        w   += [Xk]
        #          x         y       phi     delta   vx theta
        lbw += [-cd.inf, -cd.inf, -cd.inf, -cd.pi/4,  0, 0]
        ubw += [ cd.inf,  cd.inf,  cd.inf,  cd.pi/4,  2, 1]
        x_tmp, y_tmp = xpoly(theta_tmp), ypoly(theta_tmp)
        theta_step = theta_tmp + dtheta
        phi_tmp = cd.arctan((ypoly(theta_step) - y_tmp)/(xpoly(theta_step) - x_tmp))
        w0  += [xpoly(theta_tmp), ypoly(theta_tmp), phi_tmp, 0, rd.randint(0, 200)/1000., theta_tmp]
        coord_plot += [Xk]

        # Add equality constraint
        g   += [Xk_end-Xk]
        lbg += [0, 0, 0, 0, 0, 0]
        ubg += [0, 0, 0, 0, 0, 0]
    
    # Concatenate vectors
    w = cd.vertcat(*w)
    g = cd.vertcat(*g)
    coord_plot = cd.horzcat(*coord_plot)
    u_plot = cd.horzcat(*u_plot)

    # plot sparsity
    sg = cd.sum1(g)
    sparsity = cd.jacobian(cd.jacobian(sg, w), w).sparsity()
    plt.imsave(os.path.join(cfg.out_path, 'sparsity_mpcc_rk4.png'), np.array(sparsity))

    # Create an NLP solver
    solver_opts = {}
    if cfg.log_time:
        solver_opts['ipopt.output_file'] = cfg.out_log_file
        solver_opts['ipopt.print_level'] = 5
    else:
        solver_opts['print_time'] = False
        solver_opts['ipopt.print_level'] = 0
    solver_opts['ipopt.max_cpu_time'] = .5
    solver_opts['ipopt.linear_solver'] = cfg.ipopt_solver

    warm_start_opts = {}
    warm_start_opts['ipopt.warm_start_init_point'] = 'yes'
    warm_start_opts['ipopt.mu_init'] = .0001
    warm_start_opts['ipopt.warm_start_bound_push'] = 1e-9
    warm_start_opts['ipopt.warm_start_bound_frac'] = 1e-9
    warm_start_opts['ipopt.warm_start_slack_bound_frac'] = 1e-9
    warm_start_opts['ipopt.warm_start_slack_bound_push'] = 1e-9
    warm_start_opts['ipopt.warm_start_mult_bound_push'] = 1e-9

    prob = {'f': J, 'x': w, 'g': g, 'p': cd.vertcat(xc, yc)}
    solver = cd.nlpsol('solver', 'ipopt', prob, merge_dict(solver_opts, warm_start_opts))

    nlp_solver_file = cfg.comp_bin_path

    if cfg.gen_compiled:
        solver.generate_dependencies('nlp.c')                                        
        system('gcc -fPIC -shared -O3 nlp.c -o nlp.so')
    if cfg.use_compiled:
        solver = cd.nlpsol('solver', 'ipopt', nlp_solver_file, merge_dict(solver_opts, warm_start_opts))
    # Function to get x and u trajectories from w
    trajectories = cd.Function('trajectories', [w], [coord_plot, u_plot], ['w'], ['x', 'u'])

    return solver, [w0[6:], lbw[6:], ubw[6:], lbg, ubg], trajectories