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


        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.
        mn_throttle = 0.
        mx_throttle = 0.2 # limited by accel limit
        self.throttle_controller = PID(kp, ki, kd, mn_throttle, mx_throttle)

        tau = 0.5 # cutoff frequency = 1/(2*pi*tau)
        ts = 0.02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass=vehicle_mass
        self.fuel_capacity=fuel_capacity
        self.brake_deadband=brake_deadband
        self.decel_limit=decel_limit
        self.accel_limit=accel_limit
        self.wheel_radius=wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
        	self.throttle_controller.reset()
        	return 0., 0., 0.

        # run lowpass filter on current velocity
        current_vel = self.vel_lpf.filt(current_vel)

        # throttle and brake
        # use PID controller pid.py

        # steer
        # use yaw controller

        self.steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)


        return self.throttle, self.brake, self.steer
