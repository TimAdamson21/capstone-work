#!/usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from pprint import pprint
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def handle_vix_input(input):
    if input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(input.marker_name + ' was clicked.')
    else:
        rospy.loginfo('Cannot handle this interactiveMarker event, which was ' + str(input.event_type))


def main():
    rospy.init_node('interactive_marker_demo')
    wait_for_time()
    server = InteractiveMarkerServer("simple_marker")
    marker = InteractiveMarker()
    marker.header.frame_id = "base_link"
    marker.name = "adamson_marker"
    marker.description = "Simple Click Control"
    marker.pose.position.x = 1
    marker.pose.orientation.w = 1
    print("This is my interactive marker:")
    pprint(marker)

    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)

    marker.controls.append(button_control)
    print("\nAnd this is my normal marker")
    pprint(box_marker)

    server.insert(marker, handle_vix_input)
    server.applyChanges()
    rospy.spin()

if __name__ == '__main__':
    main()