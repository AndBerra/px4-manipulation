#!/usr/bin/env python3

# Waypoint list visualizer node
# Reads waypoints.json and publishes a MarkerArray on /waypoint_markers
# with transient_local QoS so RViz gets it even if it subscribes after startup.

import json
import os
import sys

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class WaypointListVisualizer(Node):

    def __init__(self):
        super().__init__('waypoint_list_visualizer')

        # Transient local — RViz gets markers even if it subscribes late
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher_ = self.create_publisher(MarkerArray, '/waypoint_markers', qos)

        self.declare_parameter('waypoints_path', '')
        waypoints_path = self.get_parameter('waypoints_path').get_parameter_value().string_value

        if not waypoints_path:
            self.get_logger().error('Parameter "waypoints_path" not set!')
            sys.exit(1)

        if not os.path.exists(waypoints_path):
            self.get_logger().error(f'Waypoints file not found: {waypoints_path}')
            sys.exit(1)

        with open(waypoints_path, 'r') as f:
            self.waypoints = json.load(f)

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {waypoints_path}')

        # Keep republishing at 1Hz — same approach as recorder
        self._timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Line strip connecting all waypoints
        if len(self.waypoints) > 1:
            line                 = Marker()
            line.header.frame_id = 'map'
            line.header.stamp    = now
            line.ns              = 'waypoint_path'
            line.id              = 0
            line.type            = Marker.LINE_STRIP
            line.action          = Marker.ADD
            line.scale.x         = 0.05
            line.color.r         = 0.0
            line.color.g         = 1.0
            line.color.b         = 1.0
            line.color.a         = 0.8
            for wp in self.waypoints:
                p = Point()
                p.x = wp['position']['x']
                p.y = wp['position']['y']
                p.z = wp['position']['z']
                line.points.append(p)
            marker_array.markers.append(line)

        # Sphere + text label per waypoint
        for i, wp in enumerate(self.waypoints):
            x = wp['position']['x']
            y = wp['position']['y']
            z = wp['position']['z']

            # Sphere
            sphere                    = Marker()
            sphere.header.frame_id    = 'map'
            sphere.header.stamp       = now
            sphere.ns                 = 'waypoint_spheres'
            sphere.id                 = i
            sphere.type               = Marker.SPHERE
            sphere.action             = Marker.ADD
            sphere.pose.position.x    = x
            sphere.pose.position.y    = y
            sphere.pose.position.z    = z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x            = 0.5
            sphere.scale.y            = 0.5
            sphere.scale.z            = 0.5
            sphere.color.r            = 0.2
            sphere.color.g            = 0.4
            sphere.color.b            = 1.0
            sphere.color.a            = 1.0
            marker_array.markers.append(sphere)

            # Text label
            text                    = Marker()
            text.header.frame_id    = 'map'
            text.header.stamp       = now
            text.ns                 = 'waypoint_labels'
            text.id                 = i
            text.type               = Marker.TEXT_VIEW_FACING
            text.action             = Marker.ADD
            text.pose.position.x    = x
            text.pose.position.y    = y
            text.pose.position.z    = z + 0.5
            text.pose.orientation.w = 1.0
            text.scale.z            = 0.4
            text.color.r            = 1.0
            text.color.g            = 1.0
            text.color.b            = 1.0
            text.color.a            = 1.0
            text.text               = str(i + 1)
            marker_array.markers.append(text)

        self.publisher_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointListVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()