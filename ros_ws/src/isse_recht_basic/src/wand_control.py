#!/usr/bin/env python3
import rospy
import tf2_ros

from ros_ws.src.rospkg.drone import Drone

if __name__ == '__main__':
	rospy.init_node('wand_control')

    # tf "topics" used by the drone
	target = rospy.get_param('~target')
	world = rospy.get_param('~world')
	mav_id = rospy.get_param('~mav_id')
	position = rospy.get_param('~position')

	# tf "topic" of our wand
	wand = rospy.get_param('~wand')

	copter = Drone(mav_id, world, position, target)
	copter.arm()
	rospy.logwarn("drone armed")

	tfBuffer = tf2_ros.Buffer()

	while not rospy.is_shutdown():
		target = tfBuffer.lookup_transform(world, wand, rospy.Time())
		copter.fly_to(target.transform)
		# TODO wand up down arm disarm
