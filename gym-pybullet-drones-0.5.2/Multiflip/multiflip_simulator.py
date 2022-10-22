
import time
import random
import numpy as np
import pybullet as p
import sys
sys.path.append('E:\study\study\Masters\Project\gym-pybullet-drones-0.5.2\gym-pybullet-drones-0.5.2')


from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync
from multiflip_controller import MultiFlipController

DURATION = 10
"""int: The duration of the simulation in seconds."""
GUI = True
"""bool: Whether to use PyBullet graphical interface."""
RECORD = False
"""bool: Whether to save a video under /files/videos. Requires ffmpeg"""

if __name__ == "__main__":

    #### Create the ENVironment ################################
    ENV = CtrlAviary(gui=GUI, record=RECORD)
    PYB_CLIENT = ENV.getPyBulletClient()

    #### Initialize the LOGGER #################################
    LOGGER = Logger(logging_freq_hz=ENV.SIM_FREQ)

    #### Initialize the controller #############################
    CTRL = MultiFlipController(ENV)

    #### Initialize the ACTION #################################
    ACTION = {}
    OBS = ENV.reset()

    STATE = OBS["0"]["state"]
    ACTION["0"] = CTRL.compute_control_flip(current_attitude=STATE[7:10],
                                       current_attitude_rate=STATE[13:16],
                                       current_altitude=STATE[2],
                                       target_attitude=np.zeros((3,)),
                                       target_altitude=0.6
                                       )

    #### Initialize target trajectory ##########################
    TARGET_POSITION = np.array([[0, 0, 1.0] for i in range(DURATION*ENV.SIM_FREQ)])
    TARGET_VELOCITY = np.zeros([DURATION * ENV.SIM_FREQ, 3])
    TARGET_ACCELERATION = np.zeros([DURATION * ENV.SIM_FREQ, 3])

    #### Derive the target trajectory to obtain target velocities and accelerations
    TARGET_VELOCITY[1:, :] = (TARGET_POSITION[1:, :] - TARGET_POSITION[0:-1, :]) / ENV.SIM_FREQ
    TARGET_ACCELERATION[1:, :] = (TARGET_VELOCITY[1:, :] - TARGET_VELOCITY[0:-1, :]) / ENV.SIM_FREQ

    #### Run the simulation ####################################
    START = time.time()
    for i in range(0, DURATION*ENV.SIM_FREQ):


        #### Step the simulation ###################################

        #ACTION = np.reshape(ACTION, (4,))
        OBS, _, _, _ = ENV.step(ACTION)

        #### Compute control #######################################
        STATE = OBS["0"]["state"]
        print('state', STATE)
        ACTION["0"] = CTRL.compute_control_flip(current_attitude=STATE[7:10],
                                       current_attitude_rate=STATE[13:16],
                                       current_altitude=STATE[2],
                                       target_attitude=np.asarray([0.01, 0.01, 0.01]),
                                       target_altitude=0.6
                                       )
        '''ACTION["0"] = CTRL.compute_control(current_position=STATE[0:3],
                                           current_velocity=STATE[10:13],
                                           target_position=TARGET_POSITION[i, :],
                                           target_velocity=TARGET_VELOCITY[i, :],
                                           target_acceleration=TARGET_ACCELERATION[i, :]
                                           )'''

        #### Log the simulation ####################################
        LOGGER.log(drone=0, timestamp=i/ENV.SIM_FREQ, state=STATE)

        #### Printout ##############################################
        if i%ENV.SIM_FREQ == 0:
            ENV.render()

        #### Sync the simulation ###################################
        if GUI:
            sync(i, START, ENV.TIMESTEP)

    #### Close the ENVironment #################################
    ENV.close()

    #### Save the simulation results ###########################
    LOGGER.save()

    #### Plot the simulation results ###########################
    LOGGER.plot()
