#!/usr/bin/env python
import math
import signal
from time import sleep

import rospy
from isse_recht_basic.drone import Drone
from AbstractVirtualCapability import VirtualCapabilityServer
from ISSE_Copter import IsseCopter


class ISSE_Copter_Ros:
    def __init__(self):
        target = rospy.get_param('~target')
        world = rospy.get_param('~world')
        self.mav_id = rospy.get_param('~mav_id')
        position = rospy.get_param('~position')
        self.copter = Drone(self.mav_id, world, position, target)
        self.armed = False

    def fly_to(self, p):
        rospy.logwarn(f"ISSE_ROS: Flying to {p}")
        self.copter.fly_to(p[0], p[1], p[2])
        pos = self.copter.get_position()
        pos = [pos.transform.translation.x, pos.transform.translation.y, pos.transform.translation.z]
        distance = math.sqrt((p[0] - pos[0]) ** 2 + (p[1] - pos[1]) ** 2 + (p[2] - pos[2]) ** 2)

        #rospy.logwarn(f"drone {self.mav_id} flying to {p}")
        i = 0
        while distance > 0.25:
            pos = self.copter.get_position()
            pos = [pos.transform.translation.x, pos.transform.translation.y, pos.transform.translation.z]
            if i % 100 == 0:
                #rospy.logwarn(f"ISSE_ROS at position:  {pos}, trying to get to  {p}")
                i = 0
            distance = math.sqrt((p[0] - pos[0]) ** 2 + (p[1] - pos[1]) ** 2 + (p[2] - pos[2]) ** 2)
            i+=1
        rospy.logwarn(f"drone {self.mav_id} arrived at {p}: {distance}")
        #rospy.logwarn(f"drone {self.mav_id} reached {p}")

    def get_position(self):
        rospy.logwarn(f"ISSE_ROS: get Position")
        current = self.copter.get_position()
        return [current.transform.translation.x, current.transform.translation.y, current.transform.translation.z]

    def arm(self):
        rospy.logwarn(f"ISSE_ROS: Arming")
        self.copter.arm()
        self.armed = True

    def disarm(self):
        rospy.logwarn(f"ISSE_ROS: Disarming")
        try:
            self.copter.disarm()
            self.armed = False
        except Exception as e:
            rospy.logwarn("ERROR: " + repr(e))

    def get_arming_status(self):
        return self.armed


if __name__ == '__main__':
    print("PLS PRINT ME!!!!")
    rospy.init_node('rosnode')
    rate = rospy.Rate(20)

    rospy.logwarn("Starting Isse_copter ROS")
    drone = ISSE_Copter_Ros()

    rospy.logwarn("Starting server")
    server = VirtualCapabilityServer(int(rospy.get_param('~semantix_port')))

    rospy.logwarn("starting isse_copter semanticplugandplay")
    copter = IsseCopter(server)
    print("PLS PRINT ME!!!!")


    copter.functionality["arm"] = drone.arm
    copter.functionality["disarm"] = drone.disarm
    copter.functionality["SetISSECopterPosition"] = drone.fly_to
    copter.functionality["GetISSECopterPosition"] = drone.get_position
    copter.functionality["GetArmingStatus"] = drone.get_arming_status
    copter.start()
    #signal.signal(signal.SIGTERM, handler)

    while not rospy.is_shutdown() and server.running:
        rate.sleep()
        rospy.logwarn(f"Server status: {server.running}, {copter}")

