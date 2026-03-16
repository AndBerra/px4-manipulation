#!/usr/bin/env python3

# Waypoint recorder node
# Offline tool — no drone connection required.
# Use the 6DOF interactive marker in RViz to position waypoints,
# then save them to config/waypoints.json via the right-click menu.
#
# Visual feedback in RViz:
#   - Sphere at each saved waypoint (blue)
#   - Line strip connecting waypoints in order (cyan)
#   - Numbered text label above each sphere
#   - Marker description updates with waypoint count

import json
import os
import sys

from geometry_msgs.msg import Point, Pose, TransformStamped
from interactive_markers import InteractiveMarkerServer, MenuHandler
import rclpy
from rclpy.node import Node
from rosidl_runtime_py import set_message_fields
from std_msgs.msg import ColorRGBA
from tf2_ros.transform_broadcaster import TransformBroadcaster
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl,
    InteractiveMarkerFeedback, Marker, MarkerArray
)





# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------

node         = None
server       = None
menu_handler = MenuHandler()
br           = None
counter      = 0


# ---------------------------------------------------------------------------
# Feedback handler
# ---------------------------------------------------------------------------

class WaypointRecorder:

    def __init__(self, marker_array_pub, waypoints_path: str):
        self.marker_pose       = Pose()
        self.waypoints         = []
        self.marker_array_pub  = marker_array_pub
        self.waypoints_path    = waypoints_path

    # ------------------------------------------------------------------
    # Serialization
    # ------------------------------------------------------------------

    def _pose_to_dict(self, pose):
        return {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
            },
            'orientation': {
                'w': pose.orientation.w,
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
            },
        }

    def _save(self):
        os.makedirs(os.path.dirname(self.waypoints_path), exist_ok=True)
        with open(self.waypoints_path, 'w') as f:
            json.dump(self.waypoints, f, indent=2)
        node.get_logger().info(f'Saved {len(self.waypoints)} waypoints to {self.waypoints_path}')

    # ------------------------------------------------------------------
    # RViz visualization
    # ------------------------------------------------------------------

    def _publish_markers(self):
        marker_array = MarkerArray()

        # --- Line strip connecting all waypoints ---
        if len(self.waypoints) > 1:
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp    = node.get_clock().now().to_msg()
            line.ns              = 'waypoint_path'
            line.id              = 0
            line.type            = Marker.LINE_STRIP
            line.action          = Marker.ADD
            line.scale.x         = 0.05          # line width
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

        # --- Sphere + text label per waypoint ---
        for i, wp in enumerate(self.waypoints):
            x = wp['position']['x']
            y = wp['position']['y']
            z = wp['position']['z']

            # Sphere
            sphere               = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp  = node.get_clock().now().to_msg()
            sphere.ns            = 'waypoint_spheres'
            sphere.id            = i
            sphere.type          = Marker.SPHERE
            sphere.action        = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x       = 0.2
            sphere.scale.y       = 0.2
            sphere.scale.z       = 0.2
            sphere.color.r       = 0.2
            sphere.color.g       = 0.4
            sphere.color.b       = 1.0
            sphere.color.a       = 1.0
            marker_array.markers.append(sphere)

            # Text label
            text               = Marker()
            text.header.frame_id = 'map'
            text.header.stamp  = node.get_clock().now().to_msg()
            text.ns            = 'waypoint_labels'
            text.id            = i
            text.type          = Marker.TEXT_VIEW_FACING
            text.action        = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = z + 0.3   # float above sphere
            text.pose.orientation.w = 1.0
            text.scale.z       = 0.25         # text height
            text.color.r       = 0.0
            text.color.g       = 0.0
            text.color.b       = 0.0
            text.color.a       = 1.0
            text.text          = str(i + 1)
            marker_array.markers.append(text)

        self.marker_array_pub.publish(marker_array)

    def _clear_markers(self):
        """Publish a DELETE_ALL to wipe all waypoint markers from RViz."""
        marker_array         = MarkerArray()
        delete_all           = Marker()
        delete_all.header.frame_id = 'map'
        delete_all.header.stamp    = node.get_clock().now().to_msg()
        delete_all.action    = Marker.DELETEALL
        marker_array.markers.append(delete_all)
        self.marker_array_pub.publish(marker_array)

    def _update_marker_description(self):
        """Update the interactive marker description with current waypoint count."""
        int_marker = server.get('waypoint_marker')
        if int_marker is not None:
            int_marker.description = f'Waypoint Recorder  [{len(self.waypoints)} pts]'
            server.insert(int_marker, feedback_callback=self.processFeedback)
            menu_handler.apply(server, 'waypoint_marker')
            server.applyChanges()

    # ------------------------------------------------------------------
    # Menu feedback
    # ------------------------------------------------------------------

    def processFeedback(self, feedback):

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.marker_pose = feedback.pose

        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            entry = feedback.menu_entry_id

            if entry == 1:    # Add Waypoint
                wp = self._pose_to_dict(self.marker_pose)
                self.waypoints.append(wp)
                node.get_logger().info(
                    f'Waypoint {len(self.waypoints)} added:\n'
                    f'  position:    x={wp["position"]["x"]:.3f}'
                    f'  y={wp["position"]["y"]:.3f}'
                    f'  z={wp["position"]["z"]:.3f}\n'
                    f'  orientation: w={wp["orientation"]["w"]:.3f}'
                    f'  x={wp["orientation"]["x"]:.3f}'
                    f'  y={wp["orientation"]["y"]:.3f}'
                    f'  z={wp["orientation"]["z"]:.3f}')
                self._publish_markers()
                self._update_marker_description()

            elif entry == 2:  # Undo Last Waypoint
                if not self.waypoints:
                    node.get_logger().warn('No waypoints to undo')
                    return
                removed = self.waypoints.pop()
                node.get_logger().info(
                    f'Removed waypoint {len(self.waypoints) + 1}, '
                    f'{len(self.waypoints)} remaining')
                self._clear_markers()
                self._publish_markers()
                self._update_marker_description()

            elif entry == 3:  # Save Waypoints
                if not self.waypoints:
                    node.get_logger().warn('No waypoints to save')
                    return
                self._save()

            elif entry == 4:  # Clear Waypoints
                self.waypoints = []
                self._clear_markers()
                self._update_marker_description()
                node.get_logger().info('Waypoints cleared')

            elif entry == 5:  # Save & Exit
                if self.waypoints:
                    self._save()
                else:
                    node.get_logger().warn('No waypoints recorded, exiting without saving')
                rclpy.shutdown()


# ---------------------------------------------------------------------------
# Marker helpers
# ---------------------------------------------------------------------------

def frameCallback():
    global counter
    transform = TransformStamped()
    set_message_fields(
        transform,
        {
            'header': {'frame_id': 'map', 'stamp': node.get_clock().now().to_msg()},
            'transform': {
                'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'rotation':    {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            },
            'child_frame_id': 'moving_frame',
        },
    )
    br.sendTransform(transform)
    counter += 1


def makeBox(msg):
    marker         = Marker()
    marker.type    = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.2
    marker.color.g = 0.6
    marker.color.b = 1.0
    marker.color.a = 1.0
    return marker


def makeBoxControl(msg):
    control                = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


def normalizeQuaternion(q):
    norm = q.x**2 + q.y**2 + q.z**2 + q.w**2
    s = norm**(-0.5)
    q.x *= s; q.y *= s; q.z *= s; q.w *= s


def make6DofMarker(position):
    int_marker                 = InteractiveMarker()
    int_marker.header.frame_id = 'map'
    int_marker.pose.position   = position
    int_marker.scale           = 1.0
    int_marker.name            = 'waypoint_marker'
    int_marker.description     = 'Waypoint Recorder  [0 pts]'

    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MENU

    axes = [
        ('rotate_x', 'move_x', 1.0, 0.0, 0.0),
        ('rotate_z', 'move_z', 0.0, 1.0, 0.0),
        ('rotate_y', 'move_y', 0.0, 0.0, 1.0),
    ]
    for rot_name, mov_name, ox, oy, oz in axes:
        for name, mode in [(rot_name, InteractiveMarkerControl.ROTATE_AXIS),
                           (mov_name, InteractiveMarkerControl.MOVE_AXIS)]:
            control                  = InteractiveMarkerControl()
            control.orientation.w    = 1.0
            control.orientation.x    = ox
            control.orientation.y    = oy
            control.orientation.z    = oz
            normalizeQuaternion(control.orientation)
            control.name             = name
            control.interaction_mode = mode
            control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

    server.insert(int_marker, feedback_callback=recorder.processFeedback)
    menu_handler.apply(server, int_marker.name)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('waypoint_recorder')

    marker_array_pub = node.create_publisher(MarkerArray, '/waypoint_markers', 10)

    node.declare_parameter('waypoints_path', '')
    waypoints_path = node.get_parameter('waypoints_path').get_parameter_value().string_value
    if not waypoints_path:
        node.get_logger().error('Parameter "waypoints_path" not set! Set it in the launch file.')
        sys.exit(1)

    recorder = WaypointRecorder(marker_array_pub, waypoints_path)
    br       = TransformBroadcaster(node)
    node.create_timer(0.01, frameCallback)
    server   = InteractiveMarkerServer(node, 'waypoint_recorder')

    menu_handler.insert('Add Waypoint',    callback=recorder.processFeedback)
    menu_handler.insert('Undo Last',       callback=recorder.processFeedback)
    menu_handler.insert('Save Waypoints',  callback=recorder.processFeedback)
    menu_handler.insert('Clear Waypoints', callback=recorder.processFeedback)
    menu_handler.insert('Save & Exit',     callback=recorder.processFeedback)

    make6DofMarker(Point(x=0.0, y=0.0, z=1.0))
    server.applyChanges()

    node.get_logger().info(
        f'Waypoint recorder ready. Will save to: {waypoints_path}')

    rclpy.spin(node)
    server.shutdown()