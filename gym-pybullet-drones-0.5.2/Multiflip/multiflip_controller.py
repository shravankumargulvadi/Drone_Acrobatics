
import numpy as np
import sys

sys.path.append('../')
sys.path.append('E:\study\study\Masters\Project\gym-pybullet-drones-0.5.2\gym-pybullet-drones-0.5.2')

from gym_pybullet_drones.envs.BaseAviary import BaseAviary


class MultiFlipController():
    """Controller class for multiflips"""

    ################################################################################

    def __init__(self, env: BaseAviary):
        """ Initialization of class HW1Control.

        Parameters
        ----------
        env : BaseAviary
            The PyBullet-based simulation environment.

        """
        self.g = env.G
        """float: Gravity acceleration, in meters per second squared."""
        self.mass = env.M
        """float: The mass of quad from environment."""
        self.timestep = env.TIMESTEP
        """float: Simulation and control timestep."""
        #self.kf_coeff = env.KF
        self.kf_coeff = 1.7518e-8

        """float: RPMs to force coefficient."""
        #self.km_coeff = env.KM
        self.km_coeff = 7.1834e-11
        """float: RPMs to torque coefficient."""


        self.p_coeff_position = 1.5
        """float: Proportional coefficient for position control."""
        self.d_coeff_position = 0.0
        """float: Derivative coefficient for position control."""

        self.cumulative_attitude_error = 0.0
        """ cumulative attitude error required for PI controller"""

        self.cumulative_altitude_error = 0.0
        """cumulative altitude error required for PID controller"""

        self.prev_altitude_error = 0.0
        """ previous altitude error for PID controller"""

        #self.prop_dia = env.PROP_RADIUS*2
        self.prop_dia = 42.68e-3
        """ drone propeller diameter"""


        self.reset()

    ################################################################################

    def reset(self):
        """ Resets the controller counter."""
        self.control_counter = 0

    ################################################################################
    def attitude_pi_controller(self, measured_attitude, desired_attitude):
        """attitude PI controller as per paper,
        takes in 3 attitude angles from IMU and the respective desired angles"""
        # implement anti-windup mechanism
        kp1 = 1.5e-3
        kp2 = 1.5e-3
        kp3 = 1.5e-3
        ki1 = 0.000
        ki2 = 0.000
        ki3 = 0.000
        attitude_error = desired_attitude-measured_attitude
        self.cumulative_attitude_error += attitude_error
        Kp = np.asarray([[kp1, 0, 0], [0, kp2, 0], [0, 0, kp3]])
        Ki = np.asarray([[ki1, 0, 0], [0, ki2, 0], [0, 0, ki3]])
        result = Kp@attitude_error+Ki@self.cumulative_attitude_error
        print('result in pi=', result)
        return result
    ################################################################################
    def attitude_rate_p_controller(self, measure_rate, desired_rate):
        """Proportional controller receives input from the PI controller and Gyro"""
        kp1 = 1.5e-4
        kp2 = 1.5e-4
        kp3 = 1.5e-3
        Kp = np.asarray([[kp1, 0, 0], [0, kp2, 0], [0, 0, kp3]])
        rate_error = desired_rate-measure_rate
        tau = Kp@rate_error
        print('result in p controller, tau', tau)
        return tau
    #################################################################################
    def altitude_pid_controller(self, measured_altitude, desired_altitude):
        """ PID to control drone altitude, receives desired altitude and barometer reading"""
        # implement anti-windup mechanism
        kp = 1.5e-4
        ki = 0.000
        kd = 0.00
        altitude_error = desired_altitude-measured_altitude
        self.cumulative_altitude_error += altitude_error
        error_rate = (altitude_error-self.prev_altitude_error)/self.timestep
        self.prev_altitude_error = altitude_error
        force = kp*altitude_error+ki*self.cumulative_altitude_error+kd*error_rate + self.g*self.mass #is gravity constant in the paper mass*g?
        print('result of pid, force', force)
        return force
    ####################################################################################################################
    def est_rotor_velocity(self, force_vector):
        """ map from force and torque to rotor angular velocity (squared), see paper for more details"""
        transf_mat = np.asarray([[self.kf_coeff, self.kf_coeff, self.kf_coeff, self.kf_coeff],
                                 [0, -self.kf_coeff*self.prop_dia, 0, self.kf_coeff*self.prop_dia],
                                 [-self.kf_coeff*self.prop_dia, 0, self.kf_coeff*self.prop_dia, 0],
                                 [-self.km_coeff, self.km_coeff, -self.km_coeff, self.km_coeff]])
        vel_squared = np.linalg.inv(transf_mat)@force_vector
        return vel_squared

    def compute_control_flip(self,
                        current_attitude,
                        current_attitude_rate,
                        current_altitude,
                        target_attitude,
                        target_altitude
                        ):
        """Compute the propellers' RPMs for the target state, given the current state.

        Parameters
        ----------
        current_position : ndarray
            (3,)-shaped array of floats containing global x, y, z, in meters.
        current_velocity : ndarray
            (3,)-shaped array of floats containing global vx, vy, vz, in m/s.
        target_position : ndarray
            (3,)-shaped array of float containing global x, y, z, in meters.
        target_velocity : ndarray, optional
            (3,)-shaped array of floats containing global, in m/s.
        target_acceleration : ndarray, optional
            (3,)-shaped array of floats containing global, in m/s^2.

        Returns
        -------
        ndarray
            (4,)-shaped array of ints containing the desired RPMs of each propeller.
        """
        self.control_counter += 1




        pi_result = self.attitude_pi_controller(current_attitude, target_attitude)
        tau_vec = self.attitude_rate_p_controller(current_attitude_rate, pi_result)
        force = np.asarray(self.altitude_pid_controller(current_altitude, target_altitude))
        #print("tau vec=", tau_vec, "force", force)
        tau_vec = np.reshape(tau_vec, (3, 1))
        force_vec = np.vstack((force, tau_vec))
        prop_rpm = np.sqrt(self.est_rotor_velocity(force_vec))
        print("before rpm", self.est_rotor_velocity(force_vec))
        print("Prop_rpm", prop_rpm)
        prop_rpm = np.reshape(prop_rpm, (4,))
        return prop_rpm





    def compute_control(self,
                        current_position,
                        current_velocity,
                        target_position,
                        target_velocity=np.zeros(3),
                        target_acceleration=np.zeros(3),
                        ):
        """Compute the propellers' RPMs for the target state, given the current state.

        Parameters
        ----------
        current_position : ndarray
            (3,)-shaped array of floats containing global x, y, z, in meters.
        current_velocity : ndarray
            (3,)-shaped array of floats containing global vx, vy, vz, in m/s.
        target_position : ndarray
            (3,)-shaped array of float containing global x, y, z, in meters.
        target_velocity : ndarray, optional
            (3,)-shaped array of floats containing global, in m/s.
        target_acceleration : ndarray, optional
            (3,)-shaped array of floats containing global, in m/s^2.

        Returns
        -------
        ndarray
            (4,)-shaped array of ints containing the desired RPMs of each propeller.
        """
        self.control_counter += 1



        ##### Calculate position and velocity errors ###############
        current_pos_error = target_position[2] - current_position[2]
        current_vel_error = target_velocity[2] - current_velocity[2]

        #### Calculate input with a PD controller ##################
        # u = desired_acceleration + Kv * velocity_error + Kp * position_error
        u = target_acceleration[2] \
            + self.d_coeff_position * current_vel_error \
            + self.p_coeff_position * current_pos_error

        ##### Calculate propeller turn rates given the PD input ####
        # turn_rate = sqrt( (m*u + m*g) / (4*Kf) )
        propellers_rpm = np.sqrt((u * self.mass + self.g * self.mass) / (4 * self.kf_coeff))

        # For up-down motion, assign the same turn rates to all motors
        propellers_0_and_3_rpm, propellers_1_and_2_rpm = propellers_rpm, propellers_rpm


        #### Print relevant output #################################
        if self.control_counter % (1 / self.timestep) == 0:
            print("current_position", current_position)
            print("current_velocity", current_velocity)
            print("target_position", target_position)
            print("target_velocity", target_velocity)
            print("target_acceleration", target_acceleration)

        return np.array([propellers_0_and_3_rpm, propellers_1_and_2_rpm,
                         propellers_1_and_2_rpm, propellers_0_and_3_rpm])
