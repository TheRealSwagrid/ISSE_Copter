#!/usr/bin/env python3
import rospy

from isse_recht_basic.drone import Drone
from time import sleep


if __name__ == '__main__':
	rospy.init_node('copter_node')
	target = rospy.get_param('~target')
	world = rospy.get_param('~world')
	mav_id = rospy.get_param('~mav_id')
	position = rospy.get_param('~position')
	copter = Drone(mav_id, world, position, target)
	copter.arm()
	rospy.logwarn("drone armed")
	# fly to points
	sleep(5)
	copter.fly_to(0, 0, 1)
	sleep(10)
	rospy.logwarn("reached target1")
	copter.fly_to(0, 0, 0)
	sleep(5)
	rospy.logwarn("reached target2")
	# disarm drone
	copter.disarm()
	rospy.logwarn("drone disarmed")
