import numpy as np
from race_car import RaceCar
from visualization import animate_trajectory, overlay_trajectory

if __name__ == '__main__':
    params = {'car_l': 0.3303,  # meters #'lf': 0.15875, 'lr': 0.17145 according to f1tenth_gym
              'car_h': 0.145,  # meters
              'car_mass': 2.4192,  # kg
              'wheel_mass': 0.1802,  # kg
              'axis_mass': 0.1,  # kg #TODO
              'axis_radius': 0.01115,  # meters
              'wheel_radius': 0.04826,  # meters
              'distance_cm': 0.02,  # meters: distance between the center of mass and the
              'gravity': 9.81,  # m/s^2
              'distance': 1.0,  # meters: distance between the ramps
              'initial_angle': np.pi / 4  # radians, angle of the ramp
              }
    v = 0.5  # m/s, control input
    car = RaceCar(params)
    animate_trajectory(car, v)
    overlay_trajectory(car, v)