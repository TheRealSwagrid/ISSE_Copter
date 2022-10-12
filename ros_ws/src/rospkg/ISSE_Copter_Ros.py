#!/usr/bin/env python
import math
import signal

import rospy
from drone import Drone
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
        self.copter.fly_to(p[0], p[1], p[2])
        pos = self.copter.get_position()
        pos = [pos.transform.translation.x, pos.transform.translation.y, pos.transform.translation.z]
        distance = math.sqrt((p[0] - pos[0]) ** 2 + (p[1] - pos[1]) ** 2 + (p[2] - pos[2]) ** 2)

        rospy.logwarn(f"drone {self.mav_id} flying to {p}")
        while distance > 0.15:
            pos = self.copter.get_position()
            pos = [pos.transform.translation.x, pos.transform.translation.y, pos.transform.translation.z]
            distance = math.sqrt((p[0] - pos[0]) ** 2 + (p[1] - pos[1]) ** 2 + (p[2] - pos[2]) ** 2)
            #  rospy.logwarn(f"drone {self.mav_id} distance to {p}: {distance}")
        rospy.logwarn(f"drone {self.mav_id} reached {p}")

    def get_position(self):
        current = self.copter.get_position()
        return [current.transform.translation.x, current.transform.translation.y, current.transform.translation.z]

    def arm(self):
        self.copter.arm()
        self.armed = True

    def disarm(self):
        self.copter.disarm()
        self.armed = False

    def get_arming_status(self):
        return self.armed


if __name__ == '__main__':
    # Needed for properly closing when process is being stopped with SIGTERM signal
    def handler(signum, frame):
        print("[Main] Received SIGTERM signal")
        copter.kill()
        quit(1)

    try:
        rospy.init_node('rosnode')
        rate = rospy.Rate(20)
        server = VirtualCapabilityServer()
        drone = ISSE_Copter_Ros()
        copter = IsseCopter(server)
        copter.functionality["arm"] = drone.arm
        copter.functionality["disarm"] = drone.disarm
        copter.functionality["SetISSECopterPosition"] = drone.fly_to
        copter.functionality["GetISSECopterPosition"] = drone.get_position
        copter.functionality["GetArmingStatus"] = drone.get_arming_status
        copter.start()
        signal.signal(signal.SIGTERM, handler)

        rospy.logwarn("arm")
        copter.SetArmingStatus({"SimpleBooleanParameter": True})
        rospy.logwarn("armed")
        copter.SetISSECopterPosition({"Position3D": [0.0, 0.0, 1]})
        copter.SetISSECopterPosition({"Position3D": [0.0, 0.0, 0]})
        rospy.logwarn("disarm")
        copter.SetArmingStatus({"SimpleBooleanParameter": False})
        rospy.logwarn("disarmed")

        while not rospy.is_shutdown():
            rate.sleep()
        copter.join()
    # Needed for properly closing, when program is being stopped wit a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
        server.kill()
        copter.kill()
