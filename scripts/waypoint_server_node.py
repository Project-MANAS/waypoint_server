#!/usr/bin/env python

# Created By Sarathkrishnan Ramesh on 22/01/2019

from __future__ import print_function
from math import radians, sin, cos, sqrt, atan2, pi, sqrt, pow
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import rospy
import json
import os
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from waypoint_server.srv import *
from std_srvs.srv import *
import tf2_ros
import tf2_geometry_msgs
from geopy import distance
from tf.transformations import euler_from_quaternion

class GeoWaypoint(object):
    def __init__(self, lat=0.0, lon=0.0, alt=0.0):
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def haversine_distance(self, target):
        origin = (self.lat, self.lon)
        wp = (target.lat, target.lon)
        return distance.distance(origin, wp).m

    def bearing(self, target):
        lat_wp, lon_wp, lat_origin, lon_origin = map(
            radians, [target.lat, target.lon, self.lat, self.lon])
        d_lon = lon_wp - lon_origin
        return atan2(
            sin(d_lon) * cos(lat_wp),
            ((cos(lat_origin) * sin(lat_wp)) -
             (sin(lat_origin) * cos(lat_wp) * cos(d_lon))))


class PoseWaypoint(object):
    def __init__(self, x=0.0, y=0.0, z=0.0, frame="map", time=None):
        self.x = x
        self.y = y
        self.z = z
        self.frame = frame
        self.orientation = None
        self.time = time if time is not None else rospy.Time.now()

    def euclidean_distance(self, target):
        return sqrt(pow(target.x - self.x, 2) + pow(target.y - self.y, 2) + pow(target.z - self.z, 2))


class WaypointServer(object):
    def __init__(self):
        self.origin_geo = None
        self.current_pos = None
        self.origin_pos = None
        self.initial_orientation = None

        self.target_wp = None
        self.target_wp_list = []
        self.wp_num = 0
        self.gps_fix = False

        self.threshold_distance = rospy.get_param("/waypoint_server/threshold_distance", 1.0)
        self.target_frame = rospy.get_param("/waypoint_server/target_frame", "map")

        self.publish_disp_from_wp = rospy.get_param("/waypoint_server/publish_displacement_from_wp", False)
        self.generate_wp_from_file = rospy.get_param("/waypoint_server/generate_waypoints_from_file", False)

        self.gps_topic = rospy.get_param("/waypoint_server/gps_topic", "gps")
        self.odom_topic = rospy.get_param("/waypoint_server/odom_topic", "odom")
        self.imu_topic =  rospy.get_param("waypoint_server/imu_topic","imu")

        self.nav_goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher('waypoint_marker', Marker, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        if self.publish_disp_from_wp:
            self.dist_pub = rospy.Publisher(
                'displacement_to_waypoint', Float32, queue_size=10)

    def read_waypoints_from_file(self, file):
        if os.path.exists(file):
            rospy.loginfo("Reading waypoints from [{}]".format(file))
            with open(file) as f:
                coordinates = json.load(f)
            self.target_wp_list = [self.geo_to_pose(GeoWaypoint(coordinate[0], coordinate[1])) for coordinate in
                                   coordinates]
        else:
            rospy.loginfo("File [{}] does not exist!".format(file))

    def run_server(self):
        rospy.loginfo("Waypoint Server is ready!")
        rate = rospy.Rate(0.5)
        rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_subscriber)
        rospy.Subscriber(self.odom_topic, Odometry, self.robot_pose_subscriber)
        rospy.Subscriber(self.imu_topic, Imu, self.robot_orientation_subscriber)
        rospy.Service('set_pose_waypoint', SetPoseWaypoint, self.set_pose_waypoint)
        rospy.Service('set_geo_waypoint', SetGeoWaypoint, self.set_geo_waypoint)
        rospy.Service('get_target_waypoint', QueryTargetWaypoint, self.get_target_waypoint)
        rospy.Service('set_last_waypoint',Trigger,self.set_last_waypoint)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.gps_fix and self.generate_wp_from_file:
                self.read_waypoints_from_file(self.generate_wp_from_file)
                self.generate_wp_from_file = False
            self.waypoint_publisher()

    def waypoint_publisher(self):
        if self.target_wp is not None:
            dist_to_wp = self.current_pos.euclidean_distance(self.target_wp)
            if dist_to_wp <= self.threshold_distance:
                self.target_wp = None
                self.waypoint_publisher()

        elif self.target_wp_list:
            self.target_wp = self.target_wp_list.pop(0)
            self.wp_num += 1
            self.pose_publisher(self.target_wp)

        else:
            rospy.loginfo("No waypoint available!")

    def geo_to_pose(self, g):
        if self.gps_fix:
            bearing_to_wp = self.origin_geo.bearing(g)
            distance_to_wp = self.origin_geo.haversine_distance(g)
            (roll, pitch, yaw) = euler_from_quaternion(self.initial_orientation)
            x = self.origin_pos.x + (distance_to_wp * cos(bearing_to_wp + yaw))
            y = self.origin_pos.y - (distance_to_wp * sin(bearing_to_wp + yaw))
            z = -g.alt - self.origin_pos.z
            rospy.loginfo(str(self.origin_pos.x)+" "+str(self.origin_pos.y)+" "+str((distance_to_wp * cos(bearing_to_wp)))+" "+str((distance_to_wp * sin(bearing_to_wp))))
            return PoseWaypoint(x, y, z, "odom")

    def transform_pose(self, pose, target_frame):
        desired_pose = PoseStamped()
        desired_pose.header.frame_id = pose.frame
        desired_pose.header.stamp = pose.time
        desired_pose.pose.position.x = pose.x
        desired_pose.pose.position.y = pose.y
        desired_pose.pose.position.z = pose.z
        desired_pose.pose.orientation.x = 0
        desired_pose.pose.orientation.y = 0
        desired_pose.pose.orientation.z = 0
        desired_pose.pose.orientation.w = 1

        transform = self.tf_buffer.lookup_transform(target_frame, pose.frame, pose.time,
                                                    rospy.Duration(1))
        transformed_pose = tf2_geometry_msgs.do_transform_pose(desired_pose, transform)
        return transformed_pose

    def pose_publisher(self, pose):
        transformed_pose = self.transform_pose(pose, self.target_frame)
        self.nav_goal_pub.publish(transformed_pose)
        self.marker_publisher(transformed_pose)

        rospy.loginfo(
            "GPS Fix is Valid! Setting Navigation Goal to: (X: %f, Y: %f, Z: %f)",
            transformed_pose.pose.position.x, transformed_pose.pose.position.y,
            transformed_pose.pose.position.z)
        rospy.loginfo(
            "Target waypoints in list: %d", len(self.target_wp_list))

    def marker_publisher(self, desired_pose):
        wp_marker = Marker()
        wp_marker.header.frame_id = desired_pose.header.frame_id
        wp_marker.header.stamp = desired_pose.header.stamp
        wp_marker.ns = "bezier"
        wp_marker.action = wp_marker.ADD
        wp_marker.type = wp_marker.SPHERE
        wp_marker.id = self.wp_num

        wp_marker.pose.position.x = desired_pose.pose.position.x
        wp_marker.pose.position.y = desired_pose.pose.position.y
        wp_marker.pose.position.z = desired_pose.pose.position.z

        wp_marker.scale.x = 1
        wp_marker.scale.y = 1
        wp_marker.scale.z = 0.1

        wp_marker.color.r = 1.0
        wp_marker.color.g = 0
        wp_marker.color.b = 0
        wp_marker.color.a = 1.0

        self.marker_pub.publish(wp_marker)

    def gps_subscriber(self, gps_msg):
        if gps_msg.status.status > -1 and not self.gps_fix and self.origin_pos is not None:
            # TODO: check if origin_pos requires transform to take into account the time difference
            self.origin_geo = GeoWaypoint(gps_msg.latitude, gps_msg.longitude)
            self.gps_fix = True
            rospy.loginfo(
                "GPS Fix Available. Origin set to Latitude: %f, Longitude: %f",
                self.origin_geo.lat, self.origin_geo.lon)

    def robot_orientation_subscriber(self, orientation_msg):
        self.initial_orientation = [
                                    orientation_msg.orientation.x,
                                    orientation_msg.orientation.y,
                                    orientation_msg.orientation.z,
                                    orientation_msg.orientation.w
                                    ]

    def robot_pose_subscriber(self, pose_msg):
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        z = pose_msg.pose.pose.position.z
        frame = pose_msg.header.frame_id
        time = pose_msg.header.stamp

        if not self.gps_fix:
            self.origin_pos = PoseWaypoint(x, y, z, frame, time)
            self.origin_pos.orientation = pose_msg.pose.pose.orientation

        self.current_pos = PoseWaypoint(x, y, z, frame, time)

        if self.publish_disp_from_wp and self.target_wp is not None:
            dist_msg = Float32()
            dist_msg.data = self.current_pos.euclidean_distance(self.target_wp)
            self.dist_pub.publish(dist_msg)

    def set_pose_waypoint(self, pose_waypoint):
        x = pose_waypoint.waypoint.pose.position.x
        y = pose_waypoint.waypoint.pose.position.y
        frame = pose_waypoint.waypoint.header.frame_id
        time = pose_waypoint.waypoint.header.stamp
        pose_wp = PoseWaypoint(x, y, frame=frame, time=time)
        pose_wp = self.transform_pose(pose_wp, "odom")
        pose_wp = PoseWaypoint(pose_wp.pose.position.x, pose_wp.pose.position.y, pose_wp.pose.position.z, frame="odom", time=time)

        rospy.loginfo("Recieved Pose Waypoint (X: %f, Y: %f)", x, y)

        if pose_waypoint.mode == 0:
            self.target_wp_list.append(pose_wp)
        elif pose_waypoint.mode == 1:
            if self.target_wp is not None:
                self.target_wp_list = [pose_wp, self.target_wp] + self.target_wp_list
            else:
                self.target_wp_list = [pose_wp] + self.target_wp_list
            self.target_wp = None
        elif pose_waypoint.mode == 2:
            self.target_wp = None
            self.target_wp_list = [pose_wp]
        else:
            self.target_wp_list = []

        self.waypoint_publisher()

        return SetPoseWaypointResponse(1)

    def set_geo_waypoint(self, geo_waypoint):
        if self.gps_fix:
            lat = geo_waypoint.waypoint.latitude
            lon = geo_waypoint.waypoint.longitude
            alt = geo_waypoint.waypoint.altitude
            geo_wp = GeoWaypoint(lat, lon, alt)
            pose_wp = self.geo_to_pose(geo_wp)
            pose_wp.frame = geo_waypoint.waypoint.header.frame_id
            pose_wp.time = geo_waypoint.waypoint.header.stamp
            rospy.loginfo(
                "Recieved Geo Waypoint (lat: %f, lon: %f, alt: %f)",
                lat, lon, alt)

            if geo_waypoint.mode == 0:
                self.target_wp_list.append(pose_wp)
            elif geo_waypoint.mode == 1:
                if self.target_wp is not None:
                    self.target_wp_list = [pose_wp, self.target_wp] + self.target_wp_list
                else:
                    self.target_wp_list = [pose_wp] + self.target_wp_list
                self.target_wp = None
            elif geo_waypoint.mode == 2:
                self.target_wp = None
                self.target_wp_list = [pose_wp]
            else:
                self.target_wp_list = []

            self.waypoint_publisher()

            return SetGeoWaypointResponse(1)
        else:
            return SetGeoWaypointResponse(0)

    def get_target_waypoint(self, query):
        res = QueryTargetWaypointResponse()
        if self.target_wp is not None:
            res.waypoint.header.frame_id = self.target_wp.frame
            res.waypoint.header.stamp = self.target_wp.time
            res.waypoint.pose.position.x = self.target_wp.x
            res.waypoint.pose.position.y = self.target_wp.y
            res.status = 1
        else:
            res.status = 0
        return res

    def set_last_waypoint(self,data):
        res = TriggerResponse()
        self.pose_publisher(self.target_wp)
        res.success = True
        res.message = "Last waypoint set successfully!"
        return res

if __name__ == "__main__":
    rospy.init_node("waypoint_server", anonymous=True)
    WaypointServer().run_server()
