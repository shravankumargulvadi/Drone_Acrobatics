
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

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.param.set_value('stabilizer.controller', '2')

        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        
        cf.commander.send_setpoint(0, 0, 0, 0)
        h=0.8
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