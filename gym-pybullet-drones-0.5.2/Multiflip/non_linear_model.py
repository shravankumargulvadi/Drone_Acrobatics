import numpy as np




class FlipController():
    def __init__(self,
                 K_w,
                 K_alpha,
                 thrust,
                 k_f,
                 d,
                 k_mu) -> None:
        self.K_w = K_w
        self.K_alpha = K_alpha
        self.prev_w = np.asarray([0.0, 0.0 ,0.0])
        self.thrust = thrust
        self.map = np.asarray([[k_f, k_f, k_f, k_f],
                               [0, -k_f*d, 0, k_f*d],
                               [-k_f*d, 0, k_f*d, 0],
                               [-k_mu, k_mu, -k_mu, k_mu]])
        self.map = np.linalg.inv(self.map)
        # self.phi_g1 = phi_g1
        # self.phi_g2 = phi_g2
        # self.phi_g3 = phi_g3
        # self.beta1 = beta1
        # self.beta2 = beta2
        # self.beta3 = beta3

    # def derivative(self, 
    #                t, 
    #                phi_measured):
    #     if phi_measured <= self.phi_g1:
    #         omega_dot = self.beta1*(t-self.)
    
    def F_filter(w):
        pass
    
    def flip_controller(self,
                        w_measured, 
                        w_reference,
                        w_dot_reference):
        '''
        gyro: 3*1 for wx, wy, wz
        reference: 3*1 wrx, wry, wrz
        '''
        #w_dot_measured = self.F_filer(w_measured)
        w_dot_measured = np.zeros((3, ))
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