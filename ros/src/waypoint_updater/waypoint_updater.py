#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

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
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        #rospy.Subscriber('/obstacle_waypoint',Int32, self.obstacle_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.waypoints = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_index = -1

        #rospy.spin()
        self.loop() #we change spin with loop to have control over the update frequency

    #Based on the code explained in the first walkthrough video
    def loop (self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                # Get the closest waypoint
                #closest_waypoint_index = self.get_closest_waypoint_index()
                #self.publish_waypoints(closest_waypoint_index)
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_index(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_index = self.waypoint_tree.query([x,y],1)[1] #just to get the index with the KD tree

        #Check if closest point is behind the ego vehicle
        closest_coordinates = self.waypoints_2d[closest_index]
        previous_coordinates = self.waypoints_2d[closest_index-1]

        #Equation for hyperplane through closest coordinates
        closest_vector = np.array(closest_coordinates)
        previous_vector = np.array(previous_coordinates)
        position_vector = np.array([x,y])

        value = np.dot(closest_vector-previous_vector, position_vector-closest_vector)

        if value > 0 :
            closest_index = (closest_index + 1 ) % len(self.waypoints_2d)
        return closest_index

    # def publish_waypoints (self, closest_index):
    #     lane = Lane()
    #     lane.header = self.base_waypoints.header
    #     lane.waypoints = self.base_waypoints.waypoints[closest_index : closest_index + LOOKAHEAD_WPS]
    #     self.final_waypoints_pub.publish(lane)
    def publish_waypoints (self):
        lane = self.generate_lane()
        self.final_waypoints_pub.publish(lane)

    def generate_lane(self):
        lane = Lane()
        closest_index = self.get_closest_waypoint_index()
        furthest_index = closest_index + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_index : furthest_index]

        if self.stopline_wp_index == -1 or (self.stopline_wp_index >= furthest_index):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints,closest_index)
        return lane

    def decelerate_waypoints(self, waypoints, closest_index):
        temp = []
        for i, waypoint in enumerate(waypoints):
            point = Waypoint()
            point.pose = waypoint.pose

            stop_index = max(self.stopline_wp_index - closest_index - 2, 0)
            distance = self.distance(waypoints, i, stop_index)
            velocity = math.sqrt(MAX_DECEL * distance)#distance/math.sqrt(1+distance*distance)
            #velocity = 2*waypoint.twist.twist.linear.x*(( 1./(1. + MAX_DECEL*math.exp(-distance))))
            if velocity < 3. :
                velocity = 0.
            point.twist.twist.linear.x = min(velocity, waypoint.twist.twist.linear.x) #respecting speed limit
            temp.append(point)
        return temp

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg #it simply grabs the car position

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints #storing the received waypoints in the object
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_index = msg.data

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
