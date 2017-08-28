#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import nav_msgs.msg
import math
from pprint import pprint

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

class NavPath(object):
    def __init__(self):
        self._path = []

    def distance_between_points(self, point1, point2):
        return math.sqrt(pow((point1.x - point2.x),2) + pow((point1.y - point2.y),2) + pow((point1.z - point2.z),2))
            
    def callback(self, msg):
        first_point = True
        rospy.loginfo(msg)
        if self._path:
            last_position = self._path.pop()
            self._path.append(last_position)
            first_point = False

        print("The callback was called!")

        if first_point or NavPath.distance_between_points(self, msg.pose.pose.position, last_position) > 0.5:
            print("placing a point")
            self._path.append(msg.pose.pose.position)
            marker = Marker(
                type=Marker.LINE_STRIP,
                id=msg.pose.pose.position.x,
                lifetime=rospy.Duration(10),
                pose=Pose(Point(0,0,0), Quaternion(0,0,0,0)),
                scale=Vector3(0.2, 0.2, 0.2),
                header=Header(frame_id='odom'),
                action=Marker.ADD,
                points=self._path,
                color=ColorRGBA(0.5, 0.0, 0.0, 0.8))
            pprint(marker)
            self.publisher.publish(marker)


def main():
    rospy.init_node('trace_demo')
    wait_for_time()
    nav_path = NavPath()
    nav_path.publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 10)
    rospy.sleep(0.5)
    rospy.Subscriber('odom', nav_msgs.msg.Odometry, nav_path.callback)
    rospy.spin()

if __name__ == '__main__':
  main()