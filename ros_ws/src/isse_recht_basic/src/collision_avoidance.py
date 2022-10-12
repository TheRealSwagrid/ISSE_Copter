#!/usr/bin/env python3
import math

import rospy
import tf2_ros
import geometry_msgs.msg


class Boid:
    def __init__(self, position: geometry_msgs.msg.Vector3, vector):
        self.position = position
        self.vector = vector


def try_get_transform(buffer, base, to):
    try:
        return buffer.lookup_transform(base, to, rospy.Time())
    except:
        return None


def main():
    rospy.init_node('collision_avoidance')
    swarm_namespace = rospy.get_param('~swarm_namespace')
    world = rospy.get_param('~world')
    min_dist = rospy.get_param('~min_distance', default=0.8)

    rate = rospy.Rate(10)

    # to get and send tf messages
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        swarm = rospy.get_param(swarm_namespace)
        dic = {}

        # collect positions and targets
        for mav_id in swarm:
            try:
                position = try_get_transform(tfBuffer, world, mav_id)

                if position is not None:
                    target = try_get_transform(tfBuffer, world, swarm[mav_id])
                    vector = None

                    if target is not None:
                        # direction the drone will move
                        vector = [target.transform.translation.x - position.transform.translation.x,
                                  target.transform.translation.y - position.transform.translation.y,
                                  target.transform.translation.z - position.transform.translation.z]

                    dic[mav_id] = Boid(position.transform.translation, vector)
            except:
                # in case we can't get position or target from tf graph
                pass

        # check for possible collisions
        if len(dic) >= 2:
            count = 0
            for mav1 in swarm:
                count += 1
                for i in range(count, len(dic)):
                    pos1 = dic[mav1].position
                    pos2 = list(dic.values())[i].position
                    dist = math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2)
                    if min_dist >= dist > 0:
                        rospy.logwarn(mav1 + " is too close to " + list(swarm.keys())[i] + " distance: " + str(dist))
                    else:
                        pass
            rospy.logwarn("")
        rate.sleep()


if __name__ == '__main__':
    main()



