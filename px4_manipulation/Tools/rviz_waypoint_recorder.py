#!/usr/bin/env python3

# Waypoint recorder node
# Offline tool — no drone connection required.
# Use the 6DOF interactive marker in RViz to position waypoints,
# then save them to config/waypoints.json via the right-click menu.

import json
import os
import sys

from geometry_msgs.msg import Point, Pose, TransformStamped
from interactive_markers import InteractiveMarkerServer, MenuHandler
import rclpy
from rclpy.node import Node
from rosidl_runtime_py import set_message_fields
from tf2_ros.transform_broadcaster import TransformBroadcaster
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl,
    InteractiveMarkerFeedback, Marker
)


# Save to config/ folder sitting next to this script in the source tree
WAYPOINTS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'waypoints.json')


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

    def __init__(self):
        self.marker_pose = Pose()
        self.waypoints   = []

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
        path = os.path.realpath(WAYPOINTS_PATH)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as f:
            json.dump(self.waypoints, f, indent=2)
        node.get_logger().info(f'Saved {len(self.waypoints)} waypoints to {path}')

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

            elif entry == 2:  # Save Waypoints
                if not self.waypoints:
                    node.get_logger().warn('No waypoints to save')
                    return
                self._save()

            elif entry == 3:  # Clear Waypoints
                self.waypoints = []
                node.get_logger().info('Waypoints cleared')

            elif entry == 4:  # Save & Exit
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
    int_marker.description     = 'Waypoint Recorder'

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

    recorder = WaypointRecorder()
    br       = TransformBroadcaster(node)
    node.create_timer(0.01, frameCallback)
    server   = InteractiveMarkerServer(node, 'waypoint_recorder')

    menu_handler.insert('Add Waypoint',    callback=recorder.processFeedback)
    menu_handler.insert('Save Waypoints',  callback=recorder.processFeedback)
    menu_handler.insert('Clear Waypoints', callback=recorder.processFeedback)
    menu_handler.insert('Save & Exit',     callback=recorder.processFeedback)

    make6DofMarker(Point(x=0.0, y=0.0, z=1.0))
    server.applyChanges()

    node.get_logger().info(
        f'Waypoint recorder ready. Will save to: {os.path.realpath(WAYPOINTS_PATH)}')

    rclpy.spin(node)
    server.shutdown()