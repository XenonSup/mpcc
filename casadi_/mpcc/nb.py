# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'
# %%
from IPython import get_ipython
import numpy as np
import matplotlib.pyplot as plt
# %matplotlib qt

# %%
def gen_t(pts1, pts2):
    """ Generate a (normalized) piecewise linear distance vector
    from start coordinates to each subsequent set of coordinates
    For use as a progress-along-path variable
    Args:
    pts1 (list): x-coordinates
    pts2 (list): y-coordinates
    Returns:
    list: normalized distances
    """
    tpts = [0]
    for i, pt in enumerate(pts1):
        if i != 0:
            # Piecewise linear distance
            dist_tmp = (pts1[i] - pts1[i-1]) ** 2 + (pts2[i] - pts2[i-1]) ** 2
            tpts += [np.sqrt(dist_tmp) + tpts[-1]]
    # Normalize
    maxt = tpts[-1]
    tpts = [t/maxt for t in tpts]
    return tpts

# %%

def l_polyfit(tpts,xpts,ypts,order, method=0):

    full_tpts = np.linspace(min(tpts),max(tpts))

    if method == 0:
        # Deprecated?
        xpoly = np.polyfit(tpts,xpts,order) # Highest power first
        ypoly = np.polyfit(tpts,ypts,order)
        xcal = np.polyval(xpoly,tpts)       # Highest power first
        ycal = np.polyval(ypoly,tpts)
        xful = np.polyval(xpoly,full_tpts)
        yful = np.polyval(ypoly,full_tpts)
    
    elif method == 1:
        # New
        xpoly = np.polynomial.polynomial.Polynomial.fit(tpts,xpts,order)
        ypoly = np.polynomial.polynomial.Polynomial.fit(tpts,ypts,order)
        xc = xpoly.convert().coef # Constant first
        xy = ypoly.convert().coef
        xcal = np.polynomial.polynomial.polyval(tpts,xc) # Constant first
        ycal = np.polynomial.polynomial.polyval(tpts,xy)
        xful = np.polynomial.polynomial.polyval(full_tpts,xc)
        yful = np.polynomial.polynomial.polyval(full_tpts,xy)

    elif method == 2:
        # Wrong
        xpoly = np.polynomial.polynomial.Polynomial.fit(tpts,xpts,order)
        ypoly = np.polynomial.polynomial.Polynomial.fit(tpts,ypts,order)
        xcal = np.polynomial.polynomial.polyval(tpts,list(xpoly))
        ycal = np.polynomial.polynomial.polyval(tpts,list(ypoly))
        xful = np.polynomial.polynomial.polyval(full_tpts,list(xpoly))
        yful = np.polynomial.polynomial.polyval(full_tpts,list(ypoly))
        
    return xpoly, ypoly, xcal, ycal, xful, yful

# %%
# Clear
xpts = [0,1,2,3,4]
ypts = [0,2,6,5,4]
tpts = gen_t(xpts,ypts)
tful = np.linspace(0,1)

xmin = min(xpts)-1
xmax = max(xpts)+1
ymin = min(ypts)-1
ymax = max(ypts)+1

plt.figure(1)
plt.subplot(1,1,1)
plt.plot(-5,5,'go')

orders = [0,1,2,3,4,5,6,7]

for ind, order in enumerate(orders):
    plt.subplot(2,4,ind+1)

    xpoly, ypoly, xcal, ycal, xful, yful = l_polyfit(tpts,xpts,ypts,order,method=2)

    plt.plot(xpts,ypts, 'bx', markersize=8)
    plt.plot(xcal,ycal, 'r+', markersize=8)
    plt.plot(xful,yful, 'g.', markersize=8)
    plt.plot(xpoly(tful),ypoly(tful), 'y^', markersize=2)

    # plt.plot(tpts,ypts, 'bx', markersize=8)
    # plt.plot(tpts,ycal, 'r+', markersize=8)

    plt.grid()
    plt.xlim(xmin,xmax)
    plt.ylim(ymin,ymax)
    print(xpoly)


# %%
order = 2
coef_polyfit = np.polynomial.polynomial.polyfit(tpts,xpts,order)
poly = np.polynomial.polynomial.Polynomial.fit(tpts,xpts,order)
# np.polynomial.polynomial.Polynomial([1.5, 1, -0.75])(np.linspace(0,10))
# %%
import random as rd
import numpy as np
# Generate the w0 vector given xc, yc parameters
init_ts = [0,1,2,3,4,5]
N = 400
xc = [2,-1,0]
yc = [-3,2,0.5]
def timer_np():
    return gen_w0_np(init_ts, N, xc, yc)
def timer_no():
    return gen_w0(init_ts, N, xc, yc)

def gen_w0_np(init_ts, N, xc, yc):
    # Very slightly slower!

    w0 = np.empty((6 + N*(3+6),))
    # Initial conditions
    w0[:6] = init_ts


    for k in range(N):
        ## U_k
        w0[6 + k*(3+6): 6 + k*(3+6) + 3]  = [rd.randint(-628, 628)/1000., rd.randint(-100, 100)/1000., rd.randint(0, 100)/1000.]
        
        ## Z_{k+1}
        
        theta_tmp   = float(k)/(N-1)
        dtheta      = 0.2
        theta_step  = theta_tmp + dtheta
        
        # xpoly = np.polynomial.polynomial.Polynomial(xc[::-1])
        # ypoly = np.polynomial.polynomial.Polynomial(yc[::-1])
        x_tmp = np.polyval(xc, theta_tmp)
        y_tmp = np.polyval(yc, theta_tmp)
        x_step = np.polyval(xc, theta_step)
        y_step = np.polyval(yc, theta_step)

        phi_tmp = np.arctan2((y_step - y_tmp), (x_step - x_tmp)) #arctan2?

        w0[6 + k*(3+6) + 3: 6 + k*(3+6) + 3 + 6] = [x_tmp, y_tmp, phi_tmp, 0, rd.randint(0,200)/1000., theta_tmp]
        
    return w0
def gen_w0(init_ts,N,xc,yc):
    """regenerate the initial guess vector

    Args:
        init_ts (list): initial conditions
        N (int): Number of control intervals
        xc (list): Highest power first polynomial coefficients 
        yc (list): Highest power first polynomial coefficients
    
    Returns:
        list: Initial guess
    
    TODO: Pre-allocate & vectorize
    """
    w0 = []
    # Initial conditions
    w0 += init_ts

    for k in range(N):
        ## U_k
        w0  += [rd.randint(-628, 628)/1000., rd.randint(-100, 100)/1000., rd.randint(0, 100)/1000.]

        ## Z_{k+1}
        
        theta_tmp   = float(k)/(N-1)
        dtheta      = 0.2
        theta_step  = theta_tmp + dtheta
        
        # xpoly = np.polynomial.polynomial.Polynomial(xc[::-1])
        # ypoly = np.polynomial.polynomial.Polynomial(yc[::-1])
        x_tmp = np.polyval(xc, theta_tmp)
        y_tmp = np.polyval(yc, theta_tmp)
        x_step = np.polyval(xc, theta_step)
        y_step = np.polyval(yc, theta_step)

        phi_tmp = np.arctan2((y_step - y_tmp), (x_step - x_tmp)) #arctan2?

        w0 += [x_tmp, y_tmp, phi_tmp, 0, rd.randint(0,200)/1000., theta_tmp]
        
    return w0
def gen_wbounds(init_ts, N):
    
    lbw_temp_u = [-2*np.pi, -1, 0]
    ubw_temp_u = [ 2*np.pi,  1, 1]

    lbw_temp_z = [-np.inf, -np.inf, -np.inf, -np.pi/4,  0, 0] 
    ubw_temp_z = [ np.inf,  np.inf,  np.inf,  np.pi/4,  2, 1]
    
    lbw = np.stack((init_ts, np.tile(np.stack((lbw_temp_u, lbw_temp_z)), N)))
    ubw = np.stack((init_ts, np.tile(np.stack((ubw_temp_u, ubw_temp_z)), N)))
    return lbw, ubw


def gen_gbounds(N):
    lbg = np.zeros(6*N)
    ubg = np.zeros(6*N)
    return lbg, ubg
