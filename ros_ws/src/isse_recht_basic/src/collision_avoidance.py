#!/usr/bin/env python3
import math

import rospy
import tf2_ros
import geometry_msgs.msg


#  "If your number X falls between A and B, and you would like Y to fall between C and D" - genius at stackoverflow
def map(x, a, b, c, d):
    y = (x - a) / (b - a) * (d - c) + c
    return y


class Boid:
    def __init__(self, position: geometry_msgs.msg.Vector3, target: tf2_ros.TransformStamped):
        self.position = position
        self.target = target


def try_get_transform(buffer, base, to):
    try:
        return buffer.lookup_transform(base, to, rospy.Time())
    except:
        return None


def main():
    rospy.init_node('collision_avoidance')

    swarm_namespace = rospy.get_param('~swarm_namespace', default="swarm")
    world = rospy.get_param('~world')
    min_distance = rospy.get_param('~min_distance', default=0.5)
    force_distance = rospy.get_param('~force_distance', default=1.0)

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
                    dic[mav_id] = Boid(position.transform.translation, target)
            except:
                # in case we can't get position or target from tf graph
                pass

        # check for possible collisions
        if len(dic) >= 2:
            count = 0
            for mav_id in swarm:
                count += 1
                for i in range(count, len(dic)):
                    pos1 = dic[mav_id].position
                    pos2 = list(dic.values())[i].position

                    dx = pos1.x - pos2.x
                    dy = pos1.y - pos2.y
                    dz = pos1.z - pos2.z

                    d = math.sqrt(dx**2 + dy**2 + dz **2)

                    if d < force_distance:
                        # map distance to value in range [1, 0] -> how dangerously close are we
                        _if = pow(map(d, min_distance, force_distance, 1, 0), 2)

                        # get 'direction' of force
                        vx = -1 if dx < 0 else 1
                        vy = -1 if dy < 0 else 1
                        vz = -1 if dz < 0 else 1

                        # weighted force vector
                        a = vx * _if  # * max_vel
                        b = vy * _if  # * max_vel
                        c = vz * _if  # * max_vel

                        # change target of each copter
                        if dic[mav_id].target is not None:
                            dic[mav_id].target.transform.translation.x += a
                            dic[mav_id].target.transform.translation.y += b
                            dic[mav_id].target.transform.translation.z += c

                        if list(dic.values())[i].target is not None:
                            list(dic.values())[i].target.transform.translation.x -= a * 0.8
                            list(dic.values())[i].target.transform.translation.y -= b * 0.8
                            list(dic.values())[i].target.transform.translation.z -= c * 0.8

                    if d < min_distance:
                        rospy.logerr(f'Careful: {mav_id} and {list(dic.keys())[i]} are closer than {min_distance}!')

            for mav_id in swarm:
                broadcaster.sendTransform(dic[mav_id].target)
            rospy.logwarn("")
        rate.sleep()


if __name__ == '__main__':
    main()



