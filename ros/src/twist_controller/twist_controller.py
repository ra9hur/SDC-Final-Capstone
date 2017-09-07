from yaw_controller import YawController
from pid import PID
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.min_speed = 0.                             # assumed
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

        # Initial PID parameters: kp_ = 0.18; ki_ = 0.001; kd_ = 1.0
        self.pid = PID(0.18, 0.001, 1.0)

        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        
        # control method can take twist data as input and return throttle, brake, and steering values. 
        
        # Within this class, you can import and use the provided pid.py and lowpass.py if needed for acceleration, and yaw_controller.py for steering. 
        # Note that it is not required for you to use these, and you are free to write and import other controllers.
        
        linear_velocity = kwargs['linear_velocity']
        angular_velocity = kwargs['angular_velocity']
        current_velocity = kwargs['current_velocity']
        dbw_enabled = kwargs['dbw_enabled']                 # To invoke PID only if dbw_enabled is true




        #### Calculations for steer
        # the yaw controller returns - how much we should turn, because of the road curvature.
        # Predictive steering values from yaw_controller
        predictive_steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        # corrective steering
        # to correct position on the road: to calculate CTE (cross track error) from PID
        # PID - helps to stay on lane, recovering from CTE

        # To calculate a spline for the final_waypoints, and then determine the cte from current pose
        # Fit a polynomial to waypoints ahead of the car, then compare y from car position w.r.t. that polynomial vs y for its closest waypoint
        error = ?
        sample_time = ?
        corrective_steer = 0.
        if (dbw_enabled):
            corrective_steer = self.pid.step(error, sample_time)

        print ("X"*20)
        print ("predictive_steer ", predictive_steer)
        print ("corrective_steer ", corrective_steer)

        # Total steering = predictive + corrective
        total_steer = predictive_steer + corrective_steer


        
        
        #### Calculations for throttle
        # linear_velocity is the target velocity that is set in waypoint_updater
        speed_limit = linear_velocity * ONE_MPH                                 # conversion from mph to mps
        throttle = (speed_limit - current_velocity) / speed_limit

        # Adjusting throttle/brakes based on steering angle
        if (abs(total_steer) > 0.087 and abs(total_steer) <= 0.175):            # > 5. and <= 10. degrees
            throttle = 0.2
        else if (abs(total_steer) > 0.175 and abs(total_steer) <= 0.262):       # > 10. and <= 15. degrees
            throttle = 0.0
        else if (abs(total_steer) > 0.262 and abs(total_steer) <= 0.35):        # > 15. and <= 20. degrees
            throttle = -0.05
        else if (abs(total_steer) > 0.35):                                      # > 20. degrees
            throttle = -0.1




        #### Calculations for brake
        brake = 0.
        if throttle < 0:
            # Ref: https://github.com/udacity/self-driving-car-sim/blob/43c30490fd1230e387397139bc01432f020860ff/Assets/1_SelfDrivingCar/Prefabs/Car.prefab#L1967
            brake = 20000*abs(throttle)
            throttle = 0.

        
        
        # Return throttle, brake, steer
        return (throttle, brake, total_steer)
