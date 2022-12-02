import logging
import time
import matplotlib.pyplot as plt
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
######################################################################
def generateTrajectory(t,phi_1=180,phi_2=0,phi_3=180, W_max=1400):
    #traj generation as per the eqn:  in paper 
    # receives arguments based on the number of flips required
    gamma_1=phi_1/W_max
    beta_1=-3/4*(gamma_1**-3)*W_max
    gamma_3=phi_3/W_max
    beta_3=-3/4*(gamma_3**-3)*W_max
    delta_1=2*gamma_1
    delta_2_prime=delta_1+phi_2/W_max
    if(t<=delta_1):
        W=beta_1/3*((t-gamma_1)**3)-beta_1*(gamma_1**2)*t+beta_1*(gamma_1**3)/3
    elif(t>delta_1 and t<=delta_2_prime):
        W=W_max
    else:
        W=beta_3/3*((gamma_3+delta_2_prime-t)**3)-beta_3*(gamma_3**2)*(2*gamma_3+delta_2_prime-t)+beta_3*(gamma_3**3)/3
    return W

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        no_of_flips=input(" Enter the number of flips (1,2 or 3):")
        if no_of_flips=='1':
            h=0.8
            phi_1=180
            phi_2=0
            phi_3=180
            W_max=1400
            T_limit=0.5
        elif no_of_flips=='2':
            h=0.7
            phi_1=200
            phi_2=300
            phi_3=220
            W_max=1600
            T_limit=0.7
        elif no_of_flips=='3':
            h=0.8
            phi_1=220
            phi_2=520
            phi_3=340
            W_max=1900
            T_limit=0.85

        cf.commander.send_setpoint(0, 0, 0, 0)
        #drone take off
        for y in range(25):
            cf.commander.send_hover_setpoint(0, 0, 0, h*y / 25)
            time.sleep(0.1)


    #execute trajectory
        for y in range(5):
            #cf.commander.send_hover_setpoint(0, 0, 0, h + 0.2 * y / 10)
            cf.commander.send_setpoint(0, 0, 0, 60000)
            time.sleep(0.05)

        t_start=time.time()
        t_current=time.time()
        t=abs(t_current-t_start)
        waypoints=[]
        time_list=[]
        while t<=T_limit:
            w=generateTrajectory(t,phi_1,phi_2,phi_3,W_max)
            waypoints.append(w)
            time_list.append(t)
            #print(w)
            cf.commander.send_setpoint(w,0,0,60000)
            time.sleep(0.05)
            t_current=time.time()
            t=abs(t_current-t_start)

        #drone land:
        #1. height is absolute 
        #2. assuming the drone tracks the flip trajectory accurately it 
        # should be back to the same height as it started viz h
        for y in range(25):
            cf.commander.send_hover_setpoint(0, 0, 0, h-h*y/25)
            time.sleep(0.1)
        cf.commander.send_stop_setpoint()
        print(waypoints)
        print(time_list)
        plt.plot(time_list,waypoints)