#!/usr/bin/env python3
import time

import rospy
import math
import tf2_ros
import geometry_msgs.msg
import mavros_msgs.srv
import threading


class Drone:
    def __init__(self, mav_id, world, position, target):
        self.world_str = world
        self.waypoint_queue = []
        self.position_str = position
        self.target_str = target
        self.hz = 20
        self.rate = rospy.Rate(self.hz)
        # to get tf messages
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # try to get arm service of drone
        self.armService = None
        self.target = None

        # make a broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster()

        try:
            self.armService = rospy.ServiceProxy('/aq_' + str(mav_id) + '/mavros/cmd/arming',
                                                 mavros_msgs.srv.CommandBool)
        except rospy.ServiceException as e:
            rospy.logwarn("Creating service proxy failed: %s" % e)

        rospy.logwarn("Waiting for current position")

        # do nothing 'till we have our position
        while not rospy.is_shutdown():
            try:
                # this is the position of our drone
                self.target = self.tfBuffer.lookup_transform(self.world_str, self.position_str, rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
                self.rate.sleep()
            continue
        self.target_publishing_process = threading.Thread(target=self._approach_target)
        self.target_publishing_process.start()

    # arm drone (error if arm service was not created sucessfully)
    def arm(self):
        self.armService(True)

    # disarm drone (error if arm service was not created sucessfully)
    def disarm(self):
        self.armService(False)

    # flies to a point starting from current own position
    def fly_to_relative(self, x: float, y: float, z: float):
        try:
            current = self.tfBuffer.lookup_transform(self.world_str, self.position_str, rospy.Time())
            self.fly_to(current.transform.translation.x + x,
                        current.transform.translation.y + y,
                        current.transform.translation.z + z)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            rospy.logwarn(error)

    def get_position(self):
        try:
            current = self.tfBuffer.lookup_transform(self.world_str, self.position_str, rospy.Time())
            return current
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            rospy.logwarn(error)
            return None

    def _add_waypoint(self, transform: geometry_msgs.msg.Transform):
        self.waypoint_queue.append(transform)

    def add_waypoint(self, x: float, y: float, z: float, a: float = 0.0, b: float = 0.0, c: float = 0.0,
                     d: float = 0.0):
        transform = geometry_msgs.msg.Transform(x, y, z, a, b, c, d)
        transform.translation.x = x
        transform.translation.y = y
        transform.translation.z = z
        transform.rotation.x = a
        transform.rotation.y = b
        transform.rotation.z = c
        transform.rotation.w = d
        self._add_waypoint(transform)

    # flies to a point in global coordinate system
    def fly_to(self, x: float, y: float, z: float):
        self.target.transform.translation.x = x
        self.target.transform.translation.y = y
        self.target.transform.translation.z = z

    # flies to a point in global coordinate system
    def fly_to_pos(self, position: geometry_msgs.msg.Transform):
        self.fly_to(position.translation.x, position.translation.y, position.translation.z)

    # mostly copied from first demo
    # runs in background and publishes targets while limiting speed (hopefully)
    def _approach_target(self):
        pose = geometry_msgs.msg.TransformStamped()
        pose.header.frame_id = self.world_str
        pose.child_frame_id = self.target_str

        while not rospy.is_shutdown():
            # update timestamp and target of published transform
            if self.target is not None:
                destination_position = self.target.transform.translation
                pose.header.stamp = rospy.Time.now()

                real_target = geometry_msgs.msg.TransformStamped()
                real_target.header.frame_id = self.world_str
                real_target.child_frame_id = "real_target"
                real_target.header.stamp = rospy.Time.now()
                real_target.transform = self.target.transform
                self.broadcaster.sendTransform(real_target)

                try:
                    # drones current position
                    current = self.tfBuffer.lookup_transform(self.world_str, self.position_str, rospy.Time())
                    # real_position should be current_position
                    real_position = current.transform.translation

                    real_dist = math.sqrt(
                        (real_position.x - destination_position.x) ** 2 +
                        (real_position.y - destination_position.y) ** 2 +
                        (real_position.z - destination_position.z) ** 2)

                    pose.transform = current.transform
                    pose.transform.translation = destination_position

                    if real_dist <= 0.01:
                        time.sleep(1 / self.hz)
                    else:
                        # send next pos
                        self.broadcaster.sendTransform(pose)
                        time.sleep(1 / self.hz)

                except (
                tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
                    rospy.logwarn(error)
                    time.sleep(1 / self.hz)
            else:
                time.sleep(1 / self.hz)
