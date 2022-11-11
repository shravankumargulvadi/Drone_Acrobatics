import numpy as np




class FlipController():
    def __init__(self,
                 K_w,
                 K_alpha,
                 thrust,
                 k_f,
                 d,
                 k_mu,
                 h,
                 lamda) -> None:
        self.K_w = K_w
        self.K_alpha = K_alpha
        self.prev_w = np.asarray([0.0, 0.0 ,0.0])
        self.thrust = thrust
        self.map = np.asarray([[k_f, k_f, k_f, k_f],
                               [0, -k_f*d, 0, k_f*d],
                               [-k_f*d, 0, k_f*d, 0],
                               [-k_mu, k_mu, -k_mu, k_mu]])
        self.map = np.linalg.inv(self.map)
        self.h = h
        self.w_measured_prev = np.zeros((3,))
        self.w_measured_dot_prev = np.zeros((3,))
        self.lamda = lamda

    def F_filter(self, w_measured):
        w_measured_dot = (w_measured - self.w_measured_prev)/self.h        
        w_measured_dot = w_measured_dot*self.h*self.lamda + \
                         self.w_measured_dot_prev*(1-self.h*self.lamda)
        self.w_measured_prev = w_measured
        self.w_measured_dot_prev = w_measured_dot
        return w_measured_dot

    def flip_controller(self,
                        w_measured, 
                        w_reference,
                        w_dot_reference):
        '''
        gyro: 3*1 for wx, wy, wz
        reference: 3*1 wrx, wry, wrz
        '''
        w_dot_measured = self.F_filer(w_measured)
        w_error = w_measured - w_reference
        w_dot_error = w_dot_reference - w_dot_measured
        torques = -self.K_w@w_error.reshape((-1, 1)) - self.K_alpha@w_dot_error.reshape((-1, 1))
        total_forces = np.concatenate((torques.reshape(-1), self.thrust))
        thrusts = self.map@total_forces.reshape((-1, 1))
        return thrusts

def main():
    pass

if __name__ == '__main__':
    main()