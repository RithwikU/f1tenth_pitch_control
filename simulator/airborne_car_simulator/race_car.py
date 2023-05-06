import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
import math as m
from math import cos, sin, pi
from pid import PID
import time


class RaceCar:
    def __init__(self, params):
        self.params = params
        self.l = params['car_l']
        self.h = params['car_h']
        self.mass_car = params['car_mass']
        self.mass_wheel = params['wheel_mass']
        self.mass_axis = params['axis_mass']
        self.g = params['gravity']
        self.wheel_radius = params['wheel_radius']
        self.axis_radius = params['axis_radius']
        self.distance_cm = params['distance_cm']
        # Fixed for now; need a separate control problem if take take-off velocity as an input
        # self.take_off_v = params['take_off_velocity']
        # Distance between two slopes
        # self.distance = params['distance']
        self.angle0 = params['initial_angle']
        # self.take_off_v = np.sqrt(self.distance * self.g / np.sin(2 * self.angle0))
        self.take_off_v = params['take_off_v']
        print(f'take_off_v is {self.take_off_v}')
        # Total flight time
        self.t_flight = 2 * self.take_off_v * np.sin(self.angle0) / self.g
        self.dt = 0.001
        self.pid = PID(0.5, 1e-5, 1e-5)  # PID pitch control
        # self.last_time = time.time()

    def calculate_angular_vel(self, v):
        """
        TODO: Need to confirm if we can calculate car's cuboid moment based on parallel-axis theorem.
        https://physics.stackexchange.com/questions/734513/could-pressing-the-brakes-on-a-car-in-mid-air-affect-its-pitch-rotation
        :param v: velocity of wheels in the air, from '/drive' topic
        :return: angular velocity of the car
        """
        I_w = 2 * (self.mass_wheel * self.wheel_radius ** 2 + self.mass_axis * self.axis_radius ** 2)
        omega_w = v / self.wheel_radius
        L_w = omega_w * I_w
        I_cm = self.mass_car * (self.l ** 2 + self.h ** 2) / 12
        I_parallel_axis = I_cm + self.mass_car * self.distance_cm ** 2
        omega_car = L_w / I_parallel_axis
        return omega_car

    def get_gt_position(self, t):
        """
        Get ground-truth position of car at timestamp t in inertial frame.
        x aligns with car's x axis and y aligns with car's z axis.

        :param t: timestamp t
        :return:
        """
        x = ((self.take_off_v * t) * np.cos(self.angle0))
        y = ((self.take_off_v * t) * np.sin(self.angle0)) - ((0.5 * self.g) * (t ** 2))
        return x, y

    def get_pitch_angle(self, t, v):
        """
        Get pitch angle at timestamp t, assuming constant angular velocity of wheels.
        :param t: timestamp t
        :param v: velocity of wheels in the air, from '/drive' topic
        :return: pitch angle of the car in radians
        """
        angular_vel = self.calculate_angular_vel(v)
        return angular_vel * t

    def step(self, t, v, dt):
        """

        :param t: timestamp t
        :param v: velocity of wheels in the air, from '/drive' topic
        :return: angular velocity at t
        """
        gt_pitch = self.get_pitch_angle(t, v)
        # noise = np.random.normal(0, 0.1, 1)
        # curr_pitch = gt_pitch + noise  # This is from IMU in reality
        error = gt_pitch  # - curr_pitch
        pitch = self.pid.update(error, dt)
        return pitch

    def get_state_response(self, v, plot=False):
        """
        Generate state response of PID with control input v
        :param plot: flag to plot pitch angles
        :param v: velocity of wheels in the air, from '/drive' topic
        :return:
        """
        t_elapsed = 0
        pitch_angles = []
        prev_angle = 0
        traj = []  # (x, y, pitch, t)
        distances = []
        timestamps = []
        print(f'total flight time is {self.t_flight}s')
        while t_elapsed < self.t_flight:
            distance_x, distance_y = self.get_gt_position(t_elapsed)
            # curr_time = time.time()
            # dt = curr_time - self.last_time
            t_elapsed += self.dt  # if change to dt, too many waypoints
            angular_velocity = self.step(t_elapsed, v, self.dt)
            pitch_angle = prev_angle + self.dt * angular_velocity
            prev_angle = pitch_angle
            pitch_angles.append(pitch_angle)
            traj.append((distance_x, distance_y, pitch_angle, t_elapsed))
            distances.append(distance_x)
            timestamps.append(t_elapsed)
            # self.last_time = curr_time
        if plot:
            plt.plot(timestamps, pitch_angles)
            plt.title('pitch angles vs. t')
            plt.ylabel('Pitch angle')
            plt.xlabel('t')
            plt.show()

            plt.plot(distances, pitch_angles)
            plt.title('pitch angles vs. distance')
            plt.ylabel('Pitch angle')
            plt.xlabel('distances')
            plt.show()
        return pitch_angles, traj

    def get_distance_from_take_off_v(self):
        """
        Get range of a projectile motion based on take off velocity.
        """
        R = self.take_off_v ** 2 * sin(2 * self.angle0) / self.g
        print(f'Range of projectile motion is {R}')
        return R
