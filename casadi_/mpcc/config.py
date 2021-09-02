import casadi as cd
import os

### OPTIONS
mpc_type     = 'mpcc'
ipopt_solver = 'mumps'  # mumps OR ma57
solve_method = 'rk4'    # rk4 OR colloc 

gen_compiled = False
use_compiled = True

if gen_compiled or use_compiled:
    # Store path to compiled binary
    comp_bin_name = '_'.join([ipopt_solver, solve_method, mpc_type])
    comp_dir      = os.path.join('compiled_casadi', mpc_type, comp_bin_name, 'nlp.so')
    comp_bin_path = os.path.abspath(comp_dir)


##### BEGIN ACCESSORY #####

#prefix = '_'.join([mpc_type, ipopt_solver, solve_method])

prefix = '_'.join(['CasADi', 'mpcc', ipopt_solver, solve_method])
if use_compiled: prefix += '_compiled'

# Create directories for various outputs
curr_path = os.path.dirname(os.path.dirname(__file__)) # sorta hacky
out_path = os.path.join(curr_path, 'out_mpcc')

# os.makedirs(out_path, exist_ok=True)
# os.makedirs(os.path.join(out_path, 'log'), exist_ok=True)
# os.makedirs(os.path.join(out_path, 'time'), exist_ok=True)
# os.makedirs(os.path.join(out_path, 'time_simple'), exist_ok=True)
# os.makedirs(os.path.join(out_path, 'eval'), exist_ok=True)
# os.makedirs(os.path.join(out_path, 'path'), exist_ok=True)

out_log_file = os.path.join(out_path, 'log', '_'.join([prefix, 'out.txt']))

# Logging
log_time = False
log_simple_time = False
time_csv = os.path.join(out_path, 'time', '_'.join([prefix, 'time.csv']))
simple_time_csv = os.path.join(out_path, 'time_simple', '_'.join([prefix, 'simple_time.csv']))

log_path = False
path_csv = os.path.join(out_path, 'path', '_'.join([prefix, 'path.csv']))

anim_save_file = os.path.join(out_path, prefix + '.gif')

pred_csv = os.path.join(out_path, 'pred.csv')
true_csv = os.path.join(out_path, 'true.csv')

##### END ACCESSORY #####

# Probem Parameters

T = 10. # Time horizon
N = 40  # number of control intervals
inter_axle = 0.5   # inter-axle distance

ts = .08 # time-step
e = 0.1 # epsilon (value for when solving stops)

### Routes

# 5th-order
curve_1 = {'init_ts': [0, 0, cd.pi/3, 0, 0, 0],
           'xpts': [0, .5, 2, 3.3],
           'ypts': [0, 1, 3, 2],
           'order': 5}
          
curve_2 = {'init_ts': [3.29, 2.09, -1.66, 0, 0, 0],
           'xpts': [3.3, 2.7, 2, 3],
           'ypts': [2, .5, -1, -2],
           'order': 5}

curve_3 = {'init_ts': [3, -2, 0, 0, 0, 0],
           'xpts': [3, 3.5, 4, 3.5, 1],
           'ypts': [-2, -2.5, -3.5, -4.5, -4.5],
           'order': 5}

curve_4 = {'init_ts': [1, -4.5, 3*cd.pi/4, 0, 0, 0],
           'xpts': [1, 0, -1.5, -3, -2.5],
           'ypts': [-4.5, -3.5, -2.5, -1.5, .5],
           'order': 5}

curve_5 = {'init_ts': [-2.5, .5, cd.pi/4, 0, 0, 0],
           'xpts': [-2.5, -1.5, 0, 3, 2, 0],
           'ypts': [.5, 1.25, 2, 0, -1, 0],
           'order': 5}

curves_lst = [curve_5]
num_targets_final = len(curves_lst)