#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)         # YawController
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)         # YawController
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)       # YawController
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)   # YawController

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # Parameters required for pid and yaw_controller
        kwargs = {
            'wheel_base'        : wheel_base,
            'steer_ratio'       : steer_ratio,
            'max_lat_accel'     : max_lat_accel,
            'max_steer_angle'   : max_steer_angle
        }

        # TODO: Create `TwistController` object
        self.controller = Controller(**kwargs)

        # TODO: Subscribe to all the topics you need to
        ## To receive target linear/angular velocity, subscribe to /twist_cmd
        # Target velocities as computed by waypoint_follower
        rospy.Subscriber('/twist_cmd', TwistStamped, self.target_vel_cb)

        ## To receive current velocity data
        # Inputs from sensor fusion ?
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_vel_cb)

        ## Indicates, if the car is under dbw or driver control. Subscribe to /vehicle/dbw_enabled
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_status_cb)

        self.target_angular_vel = None
        self.target_linear_vel = None
        self.current_angular_vel = None
        self.current_linear_vel = None
        self.dbw_enabled = None
        
        #self.previous_time_step = rospy.get_rostime()
        self.previous_time_step = rospy.rostime.get_time()

        self.loop()

    
    def target_vel_cb(self, twist):
        # Roll/pitch angles are zero ?
        self.target_angular_vel = twist.twist.angular.z

        # Coordinates for linear velocity are vehicle-centered, so only the x-direction linear velocity should be nonzero
        self.target_linear_vel = twist.twist.linear.x


    def current_vel_cb(self, twist):
        # TwistStamped datatype again used for current velocity
        self.current_angular_vel = twist.twist.angular.z
        self.current_linear_vel = twist.twist.linear.x


    def dbw_status_cb(self, dbw_status):
        self.dbw_enabled = dbw_status


    def deg2rad(self, x):
        return (x * math.pi / 180)


    def rad2deg(self, x):
        return (x * 180 / math.pi)


    def loop(self):
        rate = rospy.Rate(10) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            

            # http://wiki.ros.org/rospy/Overview/Time
            # Try to calculate using Hz information
            #current_time_step = rospy.get_rostime()
            #ros_duration = current_time_step - self.previous_time_step      # nsecs
            #sample_time = ros_duration.secs + (1e-9 * ros_duration.nsecs)   # nsec to sec
            #self.previous_time_step = current_time_step
            
            current_time_step = rospy.rostime.get_time()
            sample_time = current_time_step - self.previous_time_step
            self.previous_time_step = current_time_step
            
            kwargs = {
                'target_linear_vel'       : self.target_linear_vel, 
                'target_angular_vel'      : self.target_angular_vel, 
                'current_linear_vel'       : self.current_linear_vel, 
                'current_angular_vel'      : self.current_angular_vel, 
                'sample_time'            : sample_time,
                'dbw_enabled'            : self.dbw_enabled
            }

            throttle, brake, steer = self.controller.control(**kwargs)
            
            if (throttle > 0.2):
                throttle = 0.2

            steer = self.rad2deg(steer)

            # You should only publish the control commands if dbw is enabled
            if self.dbw_enabled:
                self.publish(throttle, brake, steer)
            rate.sleep()


    def publish(self, throttle, brake, steer):
        
        # Note: dont publish throttle and break simultaneously
        if (brake == 0.):
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        if (throttle == 0.):
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
