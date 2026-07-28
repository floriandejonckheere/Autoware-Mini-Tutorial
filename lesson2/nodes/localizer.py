#!/usr/bin/env python3

import math
import rospy

from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from pyproj import CRS, Transformer, Proj

from novatel_oem7_msgs.msg import INSPVA
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped

class Localizer:
    def __init__(self):

        # Parameters
        self.undulation = rospy.get_param('undulation')
        utm_origin_lat = rospy.get_param('utm_origin_lat')
        utm_origin_lon = rospy.get_param('utm_origin_lon')

        # Internal variables
        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_utm = CRS.from_epsg(25835)
        self.utm_projection = Proj(self.crs_utm)
        self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_utm)
        self.origin_x, self.origin_y = self.transformer.transform(utm_origin_lat, utm_origin_lon)

        print(f"({self.origin_x}.{self.origin_y})")

        # Subscribers
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.transform_coordinates)

        # Publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=10)
        self.br = TransformBroadcaster()

    def transform_coordinates(self, msg):
        ## Calculate position
        # Convert to UTM
        x, y = self.transformer.transform(msg.latitude, msg.longitude)

        # Subtract origin
        x -= self.origin_x
        y -= self.origin_y

        z = msg.height - self.undulation

        ## Calculate orientation
        # Apply azimuth correction
        azimuth_correction = self.utm_projection.get_factors(msg.longitude, msg.latitude).meridian_convergence
        azimuth = math.radians(msg.azimuth - azimuth_correction)

        # Convert azimuth (CW from y-axis) to yaw (CCW from x-axis)
        yaw = self.convert_azimuth_to_yaw(azimuth)
        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)

        orientation = Quaternion(qx, qy, qz, qw)

        # Create pose message
        current_pose = PoseStamped()
        current_pose.header.stamp = msg.header.stamp
        current_pose.header.frame_id = "map"

        current_pose.pose.position.x = x
        current_pose.pose.position.y = y
        current_pose.pose.position.z = z

        current_pose.pose.orientation = orientation

        # Publish pose message
        self.current_pose_pub.publish(current_pose)

        # Calculate velocity
        velocity = math.sqrt(msg.north_velocity**2 + msg.east_velocity**2)

        # Create velocity message
        current_velocity = TwistStamped()

        current_velocity.header.stamp = msg.header.stamp
        current_velocity.header.frame_id = "base_link"

        current_velocity.twist.linear.x = velocity

        # Publish velocity message
        self.current_velocity_pub.publish(current_velocity)

        # Create transform message
        transform = TransformStamped()

        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        # Publish transform message
        self.br.sendTransform(transform)

    @staticmethod
    def convert_azimuth_to_yaw(azimuth):
        """
        Converts azimuth to yaw. Azimuth is CW angle from the north. Yaw is CCW angle from the East.
        :param azimuth: azimuth in radians
        :return: yaw in radians
        """
        yaw = -azimuth + math.pi / 2
        # Clamp within 0 to 2 pi
        if yaw > 2 * math.pi:
            yaw = yaw - 2 * math.pi
        elif yaw < 0:
            yaw += 2 * math.pi

        return yaw

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('localizer')
    node = Localizer()
    node.run()