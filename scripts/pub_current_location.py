#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


def talker():
    pub = rospy.Publisher('/vectornav/GPS', NavSatFix, queue_size = 10)
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size = 10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    ctr = 0
    while not rospy.is_shutdown():
        latlong = NavSatFix()
        odom = Odometry()
        if ctr < 30:
            latlong.latitude = 13.347537
            latlong.longitude = 74.792198
        else:
            print "Changed current location"
            latlong.latitude = 13.347581
            latlong.longitude = 74.792391
        odom.pose.pose.position.x = 0
        odom.pose.pose.position.y = 0
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x = 0
        odom.pose.pose.orientation.y = 0
        odom.pose.pose.orientation.z = 0
        odom.pose.pose.orientation.w = 1
        pub_odom.publish(odom)
        pub.publish(latlong)
        ctr += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

