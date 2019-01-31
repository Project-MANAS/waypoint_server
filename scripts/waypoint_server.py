#!/usr/bin/env python

# Created By Sarathkrishnan Ramesh on 22/01/2019

from __future__ import print_function
from math import radians, sin, cos, sqrt, atan2, pi
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import rospkg
import rospy
import json
import os
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from gmaps_waypoint import GMapsWaypoint

EARTHRADIUS = 6371000.0


def haversine_distance(lat_origin, lon_origin, lat_wp, lon_wp):
    lat_wp, lon_wp, lat_origin, lon_origin = map(
        radians, [lat_wp, lon_wp, lat_origin, lon_origin])

    a_len = pow(sin(
        (lat_wp - lat_origin) / 2), 2) + cos(lat_origin) * cos(lat_wp) * pow(
            sin((lon_wp - lon_origin) / 2), 2)

    return EARTHRADIUS * 2.0 * atan2(sqrt(a_len), sqrt(1 - a_len))


def bearing(lat_origin, lon_origin, lat_wp, lon_wp):
    lat_wp, lon_wp, lat_origin, lon_origin = map(
        radians, [lat_wp, lon_wp, lat_origin, lon_origin])
    d_lon = lon_wp - lon_origin
    return atan2(
        sin(d_lon) * cos(lat_wp),
        cos(lat_origin) * (sin(lat_wp) - sin(lat_origin)) * cos(lat_wp) *
        cos(d_lon))


class WaypointServer(object):
    def __init__(self):
        self.lat_wp = 0.0
        self.lon_wp = 0.0
        self.alt_wp = 0.0
        self.lat_origin = 0.0
        self.lon_origin = 0.0
        self.alt_origin = 0.0
        self.lat_curr = 0.0
        self.lon_curr = 0.0
        self.alt_curr = 0.0

        self.curr_pos_x = 0.0
        self.curr_pos_y = 0.0
        self.curr_pos_z = 0.0
        self.origin_pos_x = 0.0
        self.origin_pos_y = 0.0
        self.origin_pos_z = 0.0

        self.wp_num = 0
        self.dist_from_origin = 0.0
        self.disp_to_wp = 0.0

        self.debug = True
        # self.reset_all = True

        self.wp_update_state = False
        self.new_waypoint = False
        self.generate_wp = True
        self.gps_fix = False

        self.gps_validity_timeout = 10.0
        self.last_valid_fix_time = rospy.get_rostime()

        self.nav_goal_pub = rospy.Publisher(
            'move_base_simple/goal', PoseStamped, queue_size=10)
        self.dist_pub = rospy.Publisher(
            'displacementToWaypoint', Float32, queue_size=10)

        self.pkg = rospkg.RosPack().get_path('waypoint_server')

    def run_server(self):
        rate = rospy.Rate(0.5)
        rospy.Subscriber("/vectornav/GPS", NavSatFix, self.gps_subscriber)
        rospy.Subscriber("/odom", Odometry, self.robot_pose_subscriber)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.gps_fix:
                self.waypoint_publisher()

    def waypoint_publisher(self):
        self.fetch_next_waypoint()
        # if self.gps_fix_valid and self.new_waypoint:
        if self.new_waypoint:
            self.pose_publisher()
            self.new_waypoint = False

    # def gps_fix_valid(self):
    #     time = rospy.get_time()

    #     if (time - self.last_valid_fix_time.secs) < self.gps_validity_timeout:
    #         return True

    #     rospy.loginfo("GPS Fix Invalid! Last valid update was: %f seconds ago",
    #                   time - self.last_valid_fix_time.secs)
    #     return False

    def pose_publisher(self):
        desired_pose = PoseStamped()
        desired_pose.header.frame_id = "world"
        desired_pose.header.stamp = rospy.Time.now()

        if self.debug:
            rospy.loginfo("LatWP: %f, LonWP: %f", self.lat_wp, self.lon_wp)

        bearing_to_wp = bearing(self.lat_origin, self.lon_origin, self.lat_wp,
                                self.lon_wp)

        desired_pose.pose.position.x = self.origin_pos_x + (
            self.dist_from_origin * cos(bearing_to_wp))
        desired_pose.pose.position.y = self.origin_pos_y + (
            self.dist_from_origin * sin(bearing_to_wp))
        desired_pose.pose.position.z = self.alt_wp - self.origin_pos_z
        desired_pose.pose.orientation.x = 0
        desired_pose.pose.orientation.y = 0
        desired_pose.pose.orientation.z = 0
        desired_pose.pose.orientation.w = 1

        self.nav_goal_pub.publish(desired_pose)

        rospy.loginfo(
            "GPS Fix is Valid! Setting Navigation Goal to: %f, %f, %f",
            desired_pose.pose.position.x, desired_pose.pose.position.y,
            desired_pose.pose.position.z)

        rospy.loginfo("Robot is heading %f metres at a bearing of %f degrees",
                      self.disp_to_wp, (bearing_to_wp * 180 / pi + 360) % 360)

    def gps_subscriber(self, gps_msg):
        if self.generate_wp:
            GMapsWaypoint().get_gmaps_waypoint(gps_msg)
            self.generate_wp = False
        elif gps_msg.status.status > -1 and not self.gps_fix:  # and self.reset_all:
            self.last_valid_fix_time = rospy.get_time()
            self.lat_curr = gps_msg.latitude
            self.lon_curr = gps_msg.longitude
            if not self.gps_fix:
                self.lat_origin = gps_msg.latitude
                self.lon_origin = gps_msg.longitude
            if self.lat_wp == 0.0 and self.lon_wp == 0.0:
                self.lat_wp = gps_msg.latitude
                self.lon_wp = gps_msg.longitude
            self.gps_fix = True
            if self.debug:
                rospy.loginfo(
                    "GPS Fix Available. Origin set to Latitude: %f, Longitude: %f",
                    self.lat_origin, self.lon_origin)

    def robot_pose_subscriber(self, pose_msg):
        # if self.reset_all:
        #     self.origin_pos_x = pose_msg.pose.pose.position.x
        #     self.origin_pos_y = pose_msg.pose.pose.position.y
        #     self.origin_pos_z = pose_msg.pose.pose.position.z
        self.curr_pos_x = pose_msg.pose.pose.position.x
        self.curr_pos_y = pose_msg.pose.pose.position.y
        self.curr_pos_z = pose_msg.pose.pose.position.z

        bearing_to_wp = bearing(self.lat_origin, self.lon_origin, self.lat_wp,
                                self.lon_wp)

        wp_x = self.origin_pos_x + (self.dist_from_origin * cos(bearing_to_wp))
        wp_y = self.origin_pos_x + (self.dist_from_origin * cos(bearing_to_wp))
        self.disp_to_wp = sqrt(pow((wp_x - self.curr_pos_x), 2) + pow(
            (wp_y - self.curr_pos_y), 2))

        if self.wp_update_state:
            dist_msg = Float32()
            dist_msg.data = self.disp_to_wp
            self.dist_pub.publish(dist_msg)

    def fetch_next_waypoint(self):
        file = self.pkg + '/waypoint_viewer/coordinates.json'
        if os.path.exists(file):
            # self.dist_from_origin = haversine_distance(
            #     self.lat_origin, self.lon_origin, self.lat_wp, self.lon_wp)
            # print(self.lat_origin,self.lon_origin,self.lat_wp,self.lon_wp,self.dist_from_origin,self.disp_to_wp)
            if self.disp_to_wp <= 10:
                with open(file) as f:
                    coordinates = json.load(f)
                self.wp_update_state = True
                rospy.loginfo("Fetching Next WayPoint: %f,%f",
                              coordinates[self.wp_num][0],
                              coordinates[self.wp_num][1])

                self.lat_wp = coordinates[self.wp_num][0]
                self.lon_wp = coordinates[self.wp_num][1]
                self.wp_num += 1
                self.new_waypoint = True
                self.dist_from_origin = haversine_distance(
                    self.lat_origin, self.lon_origin, self.lat_wp, self.lon_wp)
        else:
            rospy.loginfo("Waypoints are not generated")


if __name__ == "__main__":
    rospy.init_node("waypoint_server", anonymous=True)
    WaypointServer().run_server()
