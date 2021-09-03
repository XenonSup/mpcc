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
        xcal = np.polyval(xpoly,tpts)
        ycal = np.polyval(ypoly,tpts)
        xful = np.polyval(xpoly,full_tpts)
        yful = np.polyval(ypoly,full_tpts)
    
    elif method == 1:
        # New
        xpoly = np.polynomial.polynomial.Polynomial.fit(tpts,xpts,order)
        ypoly = np.polynomial.polynomial.Polynomial.fit(tpts,ypts,order)
        xc = xpoly.convert().coef
        xy = ypoly.convert().coef
        xcal = np.polynomial.polynomial.polyval(tpts,xc)
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

    xpoly, ypoly, xcal, ycal, xful, yful = l_polyfit(tpts,xpts,ypts,order,method=1)

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
