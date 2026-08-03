#!/usr/bin/env python3

import rospy
import numpy as np

from shapely import MultiPoint
from shapely.geometry import Polygon
from tf2_ros import TransformListener, Buffer, TransformException
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify

from sensor_msgs.msg import PointCloud2
from autoware_mini.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point32

BLUE80P = ColorRGBA(0.0, 0.0, 1.0, 0.8)


class ClusterDetector:
    def __init__(self):
        # Parameters
        self.min_cluster_size = rospy.get_param('~min_cluster_size')
        self.output_frame = rospy.get_param('/detection/output_frame')
        self.transform_timeout = rospy.get_param('~transform_timeout')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Publishers
        self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('points_clustered', PointCloud2, self.cluster_callback, queue_size=1, buff_size=2 ** 24,
                         tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def cluster_callback(self, msg):
        data = numpify(msg)
        points = structured_to_unstructured(data[['x', 'y', 'z', 'label']], dtype=np.float32)

        if msg.header.frame_id != self.output_frame:
            try:
                transform = self.tf_buffer.lookup_transform(self.output_frame, msg.header.frame_id, msg.header.stamp,
                                                            rospy.Duration(self.transform_timeout))
            except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                rospy.logwarn("%s - %s", rospy.get_name(), e)
                return

            tf_matrix = numpify(transform.transform).astype(np.float32)
            points_copy = points.copy()
            points_copy[:, 3] = 1
            points_copy = points_copy.dot(tf_matrix.T)
            points[:, :3] = points_copy[:, :3]

        detected_object_array = DetectedObjectArray()

        detected_object_array.header.stamp = msg.header.stamp
        detected_object_array.header.frame_id = self.output_frame

        for i in range(int(max(points[:, 3]) + 1)):
            # Find all points with a correct label
            points3d = points[points[:, 3] == i]

            # Skip small clusters (fewer points than min_cluster_size)
            if len(points3d) < self.min_cluster_size:
                continue

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cluster_detector', log_level=rospy.INFO)
    node = ClusterDetector()
    node.run()
