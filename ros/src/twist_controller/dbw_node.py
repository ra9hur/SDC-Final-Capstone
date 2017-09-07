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

        self.angular_velocity = None
        self.linear_velocity = None
        self.current_velocity = None
        self.dbw_enabled = None

        self.loop()

    
    def target_vel_cb(self, twist):
        # Roll/pitch angles are zero ?
        self.angular_velocity = twist.twist.angular.z

        # Coordinates for linear velocity are vehicle-centered, so only the x-direction linear velocity should be nonzero
        self.linear_velocity = twist.twist.linear.x


    def current_vel_cb(self, twist):
        # TwistStamped datatype again used for current velocity
        self.current_velocity = twist.twist.linear.x


    def dbw_status_cb(self, dbw_status):
        self.dbw_enabled = dbw_status


    def loop(self):
        rate = rospy.Rate(10) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            
            kwargs = {
                'linear_velocity'       : self.linear_velocity, 
                'angular_velocity'      : self.angular_velocity, 
                'current_velocity'      : self.current_velocity,
                'dbw_enabled'           : self.dbw_enabled
            }

            throttle, brake, steer = self.controller.control(**kwargs)
            
            throttle = 0.3

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
