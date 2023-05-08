import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
import math as m
from pid import PID


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
        self.angle0 = params['initial_angle']
        self.phi_des = params['phi_des']
        # self.take_off_v = np.sqrt(self.distance * self.g / np.sin(2 * self.angle0))
        self.take_off_v = params['take_off_v']
        print(f'take_off_v is {self.take_off_v}')
        # Total flight time
        self.t_flight = 2 * self.take_off_v * np.sin(self.angle0) / self.g
        self.dt = 0.001
        self.pid = PID(6.0, 1e-5, 1e-5)  # PID pitch control
        self.prev_angle = self.angle0
        self.prev_v = self.take_off_v
        # Distance between two slopes
        self.distance = self.get_distance_from_take_off_v()
        # No pid control params
        self.input_v = params['input_v']
        self.prev_v_no_pid = self.take_off_v
        self.prev_angle_no_pid = self.angle0

    def calculate_angular_vel(self):
        """
        TODO: Need to confirm if we can calculate car's cuboid moment based on parallel-axis theorem.
        https://physics.stackexchange.com/questions/734513/could-pressing-the-brakes-on-a-car-in-mid-air-affect-its-pitch-rotation
        :return: angular velocity of the car
        """
        I_w = 2 * (self.mass_wheel * (self.wheel_radius ** 2) + self.mass_axis * (self.axis_radius ** 2))
        omega_w = self.prev_v / self.wheel_radius
        L_w = omega_w * I_w
        I_cm = self.mass_car * (self.l ** 2 + self.h ** 2) / 12
        # I_parallel_axis = I_cm + self.mass_car * self.distance_cm ** 2
        # omega_car = L_w / I_parallel_axis
        omega_car = L_w / I_cm
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

    def get_pitch_angle(self):
        """
        Get pitch angle at timestamp t, assuming constant angular velocity of wheels.
        :return: pitch angle of the car in radians
        """
        # v = self.step()
        omega_car = self.calculate_angular_vel()
        current_angle = self.prev_angle + omega_car * self.dt
        self.prev_angle = current_angle
        return current_angle

    def get_pitch_angle_no_pid(self):
        I_w = 2 * (self.mass_wheel * (self.wheel_radius ** 2) + self.mass_axis * (self.axis_radius ** 2))
        omega_w = self.input_v / self.wheel_radius
        L_w = omega_w * I_w
        I_cm = self.mass_car * (self.l ** 2 + self.h ** 2) / 12
        omega_car = L_w / I_cm
        current_angle = self.prev_angle_no_pid + omega_car * self.dt
        self.prev_angle_no_pid = current_angle
        return current_angle

    def step(self, pitch_angle):
        """
        Calculate the velocity to publish to '/drive.'
        """
        error = self.phi_des - pitch_angle
        v = self.pid.update(error, self.dt)
        return v

    def get_state_response(self, plot=False):
        """
        Generate state response of PID
        :param plot: flag to plot pitch angles
        :return:
        """
        t_elapsed = 0
        pitch_angles = []
        traj = []  # (x, y, pitch, t)
        distances = []
        timestamps = []
        pid_velocities = []
        pitch_angles_no_pid = []
        traj_no_pid = []
        print(f'total flight time is {self.t_flight} s')
        while t_elapsed < self.t_flight:
            distance_x, distance_y = self.get_gt_position(t_elapsed)
            t_elapsed += self.dt
            pitch_angle = self.get_pitch_angle()
            v_PID = self.step(pitch_angle)
            self.prev_v = v_PID
            traj.append([distance_x, distance_y, pitch_angle, t_elapsed])
            pid_velocities.append(v_PID)
            pitch_angles.append(pitch_angle)
            distances.append(distance_x)
            timestamps.append(t_elapsed)
        distance_y = 100.0
        t_elapsed = 0
        while distance_y >= 0:
            distance_x, distance_y = self.get_gt_position(t_elapsed)
            t_elapsed += self.dt
            # Simulate no pid state responses
            pitch_angle_no_pid = self.get_pitch_angle_no_pid()
            pitch_angles_no_pid.append(pitch_angle_no_pid)
            traj_no_pid.append([distance_x, distance_y, pitch_angle_no_pid, t_elapsed])
        if plot:
            plt.plot(timestamps, pitch_angles)
            plt.title('pitch angles vs. t')
            plt.ylabel('Pitch angle')
            plt.xlabel('t')

            plt.plot(distances, pitch_angles)
            plt.title('pitch angles vs. distance')
            plt.ylabel('Pitch angle')
            plt.xlabel('distances')
            plt.plot(timestamps, pid_velocities)
            plt.title('velocity vs. t')
            plt.ylabel('Velocity')
            plt.xlabel('t')
            plt.show()

        return pitch_angles, traj, pitch_angles_no_pid, traj_no_pid

    def get_distance_from_take_off_v(self):
        """
        Get range of a projectile motion based on take off velocity.
        """
        R = self.take_off_v ** 2 * np.sin(2 * self.angle0) / self.g
        print(f'Range of projectile motion is {R}')
        return R
