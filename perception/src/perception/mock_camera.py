
import rospy
import rosbag
from pprint import pprint
from tf import TransformListener


class MockCamera(object):
    """A MockCamera reads saved point clouds.
    """

    def __init__(self):
        self.tf = TransformListener()
        pass

    def read_cloud(self, path):
        """Returns the sensor_msgs/PointCloud2 in the given bag file.

        Args:
            path: string, the path to a bag file with a single
            sensor_msgs/PointCloud2 in it.

        Returns: A sensor_msgs/PointCloud2 message, or None if there were no
            PointCloud2 messages in the bag file.
        """

        bag = rosbag.Bag(path)
        for topic, msg, t in bag.read_messages():
            if msg is not None:
                return msg

        return None

    def read_cloud_odom(self, path):
        """Returns the sensor_msgs/PointCloud2 in the given bag file.

        Args:
            path: string, the path to a bag file with a single
            sensor_msgs/PointCloud2 in it.

        Returns: A sensor_msgs/PointCloud2 message, or None if there were no
            PointCloud2 messages in the bag file.
        """

        bag = rosbag.Bag(path)
        for topic, msg, t in bag.read_messages():
            if msg is not None:
                return self.tf.transformPointCloud("odom", msg)

        return None