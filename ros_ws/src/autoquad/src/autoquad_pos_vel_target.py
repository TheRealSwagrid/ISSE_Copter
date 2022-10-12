#!/usr/bin/env python3
import rospy

import tf2_ros
import mavros_msgs.msg

if __name__ == '__main__':
    rospy.init_node('autoquad_pos_target')

    copter_target = rospy.get_param('~copter_target')
    copter_map = rospy.get_param('~copter_map')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    target_publisher = rospy.Publisher('mavros/setpoint_raw/local', mavros_msgs.msg.PositionTarget, queue_size=1)
    rate = rospy.Rate(20.0)
    i = 0
    hasTarget = False
    rospy.loginfo("Sending target position to copter...")
    lastTrans = None

    while not rospy.is_shutdown():
        if i >= 50:
            i = 0
        i = i + 1

        try:
            trans = tfBuffer.lookup_transform(copter_map, copter_target, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            if hasTarget:
                rospy.logwarn("Lost target for copter")
                hasTarget = False
            if i >= 50:
                rospy.logwarn(error)
            rate.sleep()
            continue

        if not hasTarget:
            rospy.logwarn("Found target for copter")
            hasTarget = True

        if i >= 50:
            rospy.loginfo("Still sending target position to copter...")

        msg = mavros_msgs.msg.PositionTarget()
        msg.header = trans.header
        msg.coordinate_frame = 1
        msg.type_mask = mavros_msgs.msg.PositionTarget.IGNORE_AFX | mavros_msgs.msg.PositionTarget.IGNORE_AFY | mavros_msgs.msg.PositionTarget.IGNORE_AFZ
        msg.position = trans.transform.translation
        msg.position.z = -msg.position.z

        if lastTrans is None or (trans.header.stamp - lastTrans.header.stamp).to_sec() > 1:
            msg.velocity.x = 0
            msg.velocity.y = 0
            msg.velocity.z = 0
        else:
            if (trans.header.stamp - lastTrans.header.stamp).to_sec() == 0:
                continue
            msg.velocity.x = (trans.transform.translation.x - lastTrans.transform.translation.x) / (
                        trans.header.stamp - lastTrans.header.stamp).to_sec()
            msg.velocity.y = (trans.transform.translation.y - lastTrans.transform.translation.y) / (
                        trans.header.stamp - lastTrans.header.stamp).to_sec()
            msg.velocity.z = -(trans.transform.translation.z - lastTrans.transform.translation.z) / (
                        trans.header.stamp - lastTrans.header.stamp).to_sec()

        msg.yaw = 0
        msg.yaw_rate = 0
        target_publisher.publish(msg)
        lastTrans = trans
        # print msg.pose.position
        rate.sleep()
