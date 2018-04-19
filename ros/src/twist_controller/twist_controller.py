from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
    	wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):


        # TODO: Implement
        self.throttle = 1.
        self.brake = 0.
        self.steer = 0.

        # min_speed = 0.1
        # self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # kp = 0.3
        # ki = 0.1
        # kd = 0.
        # mn_throttle = 0.
        # mx_throttle = 0.2 # limited by accel limit
        # self.throttle_controller = PID(kp, ki, kd, mn_throttle, mx_throttle)

        # tau = 0.5 #cutoff frequency = 1/(2*pi*tau)
        # ts = 0.02 # sample time
        # self.vel_lpf

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        # throttle and brake
        # use PID controller pid.py

        # steer
        # use yaw controller



        return self.throttle, self.brake, self.steer
