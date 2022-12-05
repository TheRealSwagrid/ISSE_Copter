#!/usr/bin/env python3
import time
import rospy

import tf2_ros
import geometry_msgs.msg


def publish_pos(data):
    pbr = tf2_ros.TransformBroadcaster()
    pt = geometry_msgs.msg.TransformStamped()
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = copter_map
    pt.child_frame_id = copter_frame
    pt.transform.translation.x = data.pose.position.x
    pt.transform.translation.y = data.pose.position.y
    pt.transform.translation.z = -data.pose.position.z
    pt.transform.rotation.x = data.pose.orientation.x
    pt.transform.rotation.y = data.pose.orientation.y
    pt.transform.rotation.z = data.pose.orientation.z
    pt.transform.rotation.w = data.pose.orientation.w
    pbr.sendTransform(pt);


if __name__ == '__main__':
    rospy.init_node('autoquad_vicon_bridge')
    time.sleep(3)

    copter_frame = rospy.get_param('~copter_frame')
    copter_map = rospy.get_param('~copter_map')
    vicon_frame = rospy.get_param('~vicon_frame')
    vicon_map = rospy.get_param('~vicon_map')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.Subscriber("mavros/local_position/pose", geometry_msgs.msg.PoseStamped, publish_pos)

    mocap_publisher = rospy.Publisher('mavros/mocap/pose', geometry_msgs.msg.PoseStamped, queue_size=1)
    i = 0
    rate = rospy.Rate(20.0)
    rospy.logwarn("Waiting for copter and vicon position...")
    while not rospy.is_shutdown():
        try:
            copter_pos = tfBuffer.lookup_transform(copter_frame, copter_map, rospy.Time())
            tfBuffer.lookup_transform(vicon_map, vicon_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            i = i + 1
            if i == 50:
                rospy.logerr("Still waiting for copter and vicon position")
                rospy.logwarn(error)
                i = 0

            #            print error
            rate.sleep()
            continue

        break

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = vicon_frame
    t.child_frame_id = copter_map
    t.transform = copter_pos.transform

    rospy.logwarn("Calculating position of copter origin...")
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        br.sendTransform(t)
        try:
            trans = tfBuffer.lookup_transform(vicon_map, copter_map, rospy.Time())
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            # print error
            rate.sleep()
            continue

    rospy.logwarn("Publishing position of copter origin...")
    rospy.logwarn("Sending mocap position to copter...")
    t.header.frame_id = vicon_map
    t.child_frame_id = copter_map
    t.transform = trans.transform
    i = 0

    #    print "Received position, starting..."
    while not rospy.is_shutdown():

        t.header.stamp = rospy.Time.now()
        br.sendTransform(t)

        try:
            trans = tfBuffer.lookup_transform(copter_map, vicon_frame, rospy.Time.now())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            rospy.logerr(error)
            rate.sleep()
            continue

        i = i + 1
        if i == 50:
            rospy.logwarn("Still sending mocap position to copter...")
            i = 0
        # print trans
        msg = geometry_msgs.msg.PoseStamped()
        msg.header = trans.header
        msg.header.stamp = rospy.Time.from_sec(rospy.get_time() / 1000.0)
        msg.pose.position = trans.transform.translation
        msg.pose.orientation = trans.transform.rotation
        mocap_publisher.publish(msg)
        # print msg.pose.position
        rate.sleep()
