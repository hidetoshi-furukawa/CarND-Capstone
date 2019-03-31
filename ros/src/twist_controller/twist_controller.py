from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit
                                     , accel_limit, wheel_radius, wheel_base, steer_ratio
                                     , max_lat_accel, max_steer_angle):

        self.min_vel = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, self.min_vel, max_lat_accel, max_steer_angle)

        # Coefficients for PID Controller
        p = 0.3
        i = 0.1
        d = 0.

        min_throttle = 0.
        max_throttle = 0.2

        # initialize PID controller
        self.throttle_controller = PID(p, i, d, min_throttle, max_throttle)

        tau = 0.5
        ts = .02
        self.vel_lps = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, dbw_enabled, current_vel, linear_vel, angular_vel):
        # if dbw not enabled, reset controller
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        # filter current velocity
        current_vel = self.vel_lps.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        brake = 0.

        if linear_vel == 0. and current_vel < self.min_vel:
            throttle = 0.
            brake = 700
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0.
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
        return throttle, brake, steering
