#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.base_waypoints = None
        
        self.lane = Lane()

        self.update()
        rospy.spin()

        
    def pose_cb(self, msg):
        # TODO: Implement
        # msg.pose.position.x
        # msg.pose.orientation.w
        self.current_pose = msg
        #pass

        
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # waypoints[0].pose.pose.position.x
        # waypoints[0].twist.twist.linear.x
        
        # Set lane.header
        self.lane.header = waypoints.header
        
        self.base_waypoints = waypoints.waypoints
        #pass


    def check_car_ahead(car, wp):

        # orientation is in quaternions, need to convert them to euler
        _, _, yaw = tf.transformations.euler_from_quaternion([car.orientation.x,
                                                                car.orientation.y,
                                                                car.orientation.z,
                                                                car.orientation.w])

        delta_x = waypoint.pose.pose.position.x - car.position.x
        delta_y = waypoint.pose.pose.position.y - car.position.y

        x = delta_x * cos(0 - yaw) - delta_y * sin(0 - yaw)

        return True if (x < 0.) else False
        
    
    def next_waypoint(self):
        
        waypoints = self.base_waypoints

        min_dist = 10000    # number to be determined
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        current_pose_index = 0
        for index, wp in enumerate(waypoints):
            
            # Calculate euclidean distance
            dist = dl(self.current_pose.pose, wp.pose.pose.position)
            
            if (dist < min_dist):
                
                # Is the car ahead of the waypoint
                ahead = check_car_ahead(self.current_pose.pose, wp.pose.pose.position)
                ahead = True
                if (ahead):
                    current_pose_index = index + 1
                else:
                    current_pose_index = index
                
                break
            wp_prev = wp
        
        return current_pose_index
    
    
    def update(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if ((self.current_pose is not None) and (self.base_waypoints is not None)):

                # Compare current_pose with the poses in base_waypoints
                next_index = self.next_waypoint()

                # Set lane.waypoints to include 200 points - number of points to publish
                self.lane.waypoints = self.base_waypoints[next_index:next_index+LOOKAHEAD_WPS]

                # velocity that car should move at, when travelling through the waypoint
                target_velocity = 30.
                for wp in self.lane.waypoints:
                    set_waypoint_velocity(self.lane.waypoints, wp, target_velocity)
            
                self.final_waypoints_pub.publish(self.lane)

            rate.sleep()


    # The waypoint updater should update waypoints with the ideal velocity for each waypoint, based on desired speed and traffic lights. 
    # Control of the car (i.e. making sure that the current velocity matches the target velocity), should take place in the DBW node.
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity


    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
