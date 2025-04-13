#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovariance
import math

waypoints = [
    [0, 0, 0],
    [0, 0, 0], # automatically ignores first two messages
    
    [0, 0, 0],
    [5, 0, math.pi / 2],
    [5, 5, 5 / 4 * math.pi],
    [-5, -5, math.pi / 2],
    [-5, 5, 0],
    [0, 0, 0],
    [3, 3, 3 / 4 * math.pi],
    [-3, 0, 3 / 2 * math.pi],
    [0, -3, math.pi / 4],
    [3, 0, math.pi / 2],
    [0, 0, 3 / 2 * math.pi]
]

def publisher_pose():
    pub = rospy.Publisher('geometry_msgs/PoseWithCovariance', PoseWithCovariance, 			queue_size=50)
    rospy.init_node('publisher_Assignment_II', anonymous=False)
    rate = rospy.Rate(0.5)

    number_of_points = len(waypoints)

    if not rospy.is_shutdown():
        for i, point in enumerate(waypoints):
            msg = PoseWithCovariance()
            msg.covariance[0] = point[0]
            msg.covariance[1] = point[1]
            msg.covariance[2] = point[2]
            msg.covariance[3] = i + 1
            if i == number_of_points - 1:  # check if it's the last point
                msg.covariance[4] = 1
            else:
                msg.covariance[4] = 0
                
            pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher_pose()
    except rospy.ROSInterruptException:
        pass

