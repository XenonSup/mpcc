import random as rd
import numpy as np

class BoundAndGuess:

    def __init__(self, init_ts,N, xc, yc):
        self.coeff = False
        self.N = N

        self.regen_bounds(init_ts, xc, yc)
        self.lbg, self.ubg = self.gen_gbounds(self.N)

    def regen_bounds(self, init_ts, xc, yc):
        self.w0 = self.gen_w0(init_ts, self.N, xc, yc)
        self.lbw, self.ubw = self.gen_wbounds(init_ts, self.N)
        
    def get_bounds(self):
        return self.w0, self.lbw, self.ubw, self.lbg, self.ubg

    def get_bounds_suffix(self):
        return self.w0[6:], self.lbw[6:].tolist(), self.ubw[6:].tolist(), self.lbg, self.ubg

    def update(self, init_ts, xc, yc, suffix=False):
        self.regen_bounds(init_ts, xc, yc)
        
        if suffix:
            return self.get_bounds_suffix()
        else:
            return self.get_bounds()

    def gen_w0(self, init_ts,N,xc,yc):
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
            if self.coeff:
                x_tmp = np.polyval(xc, theta_tmp)
                y_tmp = np.polyval(yc, theta_tmp)
                x_step = np.polyval(xc, theta_step)
                y_step = np.polyval(yc, theta_step)
            else:
                x_tmp, y_tmp    = xc(theta_tmp),  yc(theta_tmp)
                x_step, y_step  = xc(theta_step), yc(theta_step)
                

            phi_tmp = np.arctan2((y_step - y_tmp), (x_step - x_tmp)) #arctan2?

            w0 += [x_tmp, y_tmp, phi_tmp, 0, rd.randint(0,200)/1000., theta_tmp]
            
        return w0
    def gen_wbounds(self, init_ts, N):
        
        lbw_temp_u = [-2*np.pi, -1, 0]
        ubw_temp_u = [ 2*np.pi,  1, 1]

        lbw_temp_z = [-np.inf, -np.inf, -np.inf, -np.pi/4,  0, 0] 
        ubw_temp_z = [ np.inf,  np.inf,  np.inf,  np.pi/4,  2, 1]
        
        # lbw = np.stack((init_ts, np.tile(np.stack((lbw_temp_u, lbw_temp_z)), N)))
        # ubw = np.stack((init_ts, np.tile(np.stack((ubw_temp_u, ubw_temp_z)), N)))
        lbw = np.concatenate((init_ts, np.tile(np.concatenate((lbw_temp_u, lbw_temp_z)), N)))
        ubw = np.concatenate((init_ts, np.tile(np.concatenate((ubw_temp_u, ubw_temp_z)), N)))
        return lbw, ubw


    def gen_gbounds(self, N):
        lbg = np.zeros(6*N)
        ubg = np.zeros(6*N)
        return lbg, ubg

##### Testing #####
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