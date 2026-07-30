#!/usr/bin/env python3

import numpy as np
import rospy
from threading import Lock

from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import Path, Waypoint

import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest


class GlobalPlanner:
    def __init__(self):
        # Parameters
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.speed_limit = float(rospy.get_param("~speed_limit"))

        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        self.output_frame = rospy.get_param("lanelet2_global_planner/output_frame")
        self.distance_to_goal_limit = rospy.get_param("lanelet2_global_planner/distance_to_goal_limit")

        # Load Lanelet2 map
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            raise RuntimeError('Only "utm" is supported for lanelet2 map loading')
        self.lanelet2_map = load(lanelet2_map_path, projector)

        # Load traffic rules and routing graph
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.VehicleTaxi)

        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        # Internal variables
        self.lock = Lock()
        self.current_location = None
        self.goal_point = None

        # Publishers
        self.global_path_pub = rospy.Publisher('global_path', Path, latch=True, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def goal_callback(self, msg):
        with self.lock:
            self.goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.current_location is None:
            return

        rospy.loginfo("%s - goal position (%f, %f, %f) in %s frame", rospy.get_name(),
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                      msg.header.frame_id)

        # Find current lanelet
        start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
        goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.goal_point, 1)[0][1]

        # Find route from start to goal lanelet (no lane changes)
        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, False)

        if route is None:
            rospy.logwarn("%s - No route found to goal position", rospy.get_name())
            return

        # Find shortest path from start to goal lanelet
        path = route.shortestPath()

        if path is None:
            rospy.logwarn("%s - No path found to goal position", rospy.get_name())
            return

        # Get path without lane changes
        path_no_lane_change = path.getRemainingLane(start_lanelet)

        # Convert path to waypoints
        waypoints = self.convert_laneletseq_to_waypoints_list(path_no_lane_change)

        # Publish waypoints
        self.publish_lane_from_waypoints_list(waypoints)

    def current_pose_callback(self, msg):
        with self.lock:
            self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.goal_point is None:
            return

        # Calculate distance to goal point
        d_ego_from_goal = np.sqrt((self.current_location.x - self.goal_point.x) ** 2 +
                                  (self.current_location.y - self.goal_point.y) ** 2)

        if d_ego_from_goal < self.distance_to_goal_limit:
            # Publish empty waypoints list
            self.publish_lane_from_waypoints_list([])
            self.goal_point = None

            rospy.loginfo("%s - goal position (%f, %f, %f) reached", rospy.get_name(),
                          msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def convert_laneletseq_to_waypoints_list(self, laneletseq):
        waypoints = []

        for j, lanelet in enumerate(laneletseq):
            # Get speed from lanelet attribute, or default to global speed limit
            speed_km_h = lanelet.attributes['speed_limit'] or self.speed_limit

            # Convert from km/h to m/s
            speed = min(float(speed_km_h), self.speed_limit) / 3.6

            # Iterate through the centerline points and create waypoints
            for i, point in enumerate(lanelet.centerline):
                # Skip first point of every lanelet except the very first (endpoints overlap)
                if i == 0 and j != 0:
                    continue

                waypoint = Waypoint()

                waypoint.position.x = point.x
                waypoint.position.y = point.y
                waypoint.position.z = point.z
                waypoint.speed = speed

                waypoints.append(waypoint)

        # TODO 5: Sync path end with goal point.
        #         The path end and goal point may not align because findNearest()
        #         returns a full lanelet. Find your own solution — see README for ideas.

        return waypoints

    def publish_lane_from_waypoints_list(self, waypoints):
        lane = Path()
        lane.header.frame_id = self.output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        self.global_path_pub.publish(lane)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('global_planner')
    node = GlobalPlanner()
    node.run()
