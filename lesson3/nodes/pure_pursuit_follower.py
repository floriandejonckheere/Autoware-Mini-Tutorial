#!/usr/bin/env python3

import numpy as np
import rospy
from threading import Lock

from autoware_mini.msg import Path, VehicleCommand
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

from shapely.geometry import LineString, Point
from shapely import prepare, distance
from scipy.interpolate import interp1d


class PurePursuitFollower:
    def __init__(self):

        # Parameters
        # TODO 4: Read in parameter values:
        #         self.lookahead_distance = rospy.get_param("~lookahead_distance")
        #         self.wheel_base = rospy.get_param("/vehicle/wheel_base")

        # Internal variables
        self.lock = Lock()
        self.path_linestring = None
        self.distance_to_velocity_interpolator = None

        # Publishers
        # TODO 2: Create a publisher for '/control/vehicle_cmd' topic with VehicleCommand message type.
        #         self.vehicle_cmd_pub = rospy.Publisher(...)

        # Subscribers
        rospy.Subscriber('path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def path_callback(self, msg):
        if len(msg.waypoints) < 2:
            path_linestring = None
            distance_to_velocity_interpolator = None
        else:
            # TODO 3: Convert waypoints to a shapely LineString and prepare it for spatial queries.
            #         - Create LineString from waypoint (x, y) coordinates
            #         - Use prepare() to create a spatial index for efficient queries

            path_linestring = None

            # TODO 5: Create a distance-to-velocity interpolator for the path.
            #         - Collect waypoint (x, y) coordinates into a numpy array
            #         - Calculate cumulative distances between waypoints
            #         - Extract velocity values from waypoints
            #         - Create an interp1d interpolator (bounds_error=False, fill_value=0.0)

            distance_to_velocity_interpolator = None

        with self.lock:
            self.path_linestring = path_linestring
            self.distance_to_velocity_interpolator = distance_to_velocity_interpolator

    def current_pose_callback(self, msg):
        if self.path_linestring is None:
            print(msg.pose.position.x, msg.pose.position.y)

            steering_angle = 0.0
            linear_velocity = 0.0
            linear_acceleration = -3.0
        else:
            # TODO 3: Calculate the ego vehicle's distance from the path start.
            #         - Convert ego position to a shapely Point
            #         - Use self.path_linestring.project() to find the distance
            #         - Remove the TODO 1 printout and print d_ego_from_path_start instead

            # TODO 4: Calculate the steering angle using the Pure Pursuit formula.
            #         - Get heading from msg.pose.orientation using euler_from_quaternion
            #         - Calculate the lookahead point using self.path_linestring.interpolate()
            #         - Calculate the lookahead heading using np.arctan2
            #         - Calculate the actual lookahead distance using shapely distance()
            #         - Apply the steering formula

            steering_angle = 0.0
            linear_velocity = 0.0
            linear_acceleration = 0.0

            # TODO 5: Use the distance-to-velocity interpolator to get the velocity
            #         at the ego vehicle's position on the path. Replace the constant
            #         linear_velocity with the interpolated value.
            #         Since the interpolator is now used here, add a check for
            #         self.distance_to_velocity_interpolator is None to the if statement above.

        # TODO 2: Create and publish a VehicleCommand message with constant steering angle and velocity for testing.

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()
