#!/usr/bin/env python
import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]
    return t

class SimpleTrack():
    def __init__(self,
            length_straight_track=1000, radius_curve_track=350, lane_width=4,
            lane_num_clockwise=2, lane_num_couterclockwise=2):
        self._length_straight_track = length_straight_track #m
        self._radius_curve_track = radius_curve_track #m
        self._lane_width = lane_width #m
        self._lane_num_clockwise = lane_num_clockwise
        self._lane_num_counterclockwise = lane_num_couterclockwise

        rospy.init_node('lane_map_manager')
        rospy.Subscriber('vehicle_position', geometry_msgs.msg.Point, self.callback_update_vehicle_perception)
        self.publisher_map = rospy.Publisher('lane_map', MarkerArray, queue_size=1)
        self.publisher_vehicle_perception = rospy.Publisher('vehicle_perception', geometry_msgs.msg.PoseArray, queue_size=1)

        self._init_map_markers()

    def callback_update_vehicle_perception(self, data):
        # TODO: update vehicle vehicle_position
        # TODO: update MarkerArray for visualization
        # TODO: update vehicle_perception message
        _pose_array = geometry_msgs.msg.PoseArray()
        self.publisher_vehicle_perception.publish(_pose_array)

    def publish_map(self):
        published=False
        while not rospy.is_shutdown():

            if published:
                break
            self.publisher_map.publish(self._map_markers)
            published=True
            rospy.sleep(1)

    def _init_map_markers(self):
        s = 0.5

        def _make_lane_markings(clockwise=True, lane_index=-1):
            _lane_marking = Marker()
            _lane_marking.header.frame_id = "/map"
            _lane_marking.ns = 'lane'
            if lane_index==-1:
                _lane_marking.id = 0 # center lane
                _deviation = 0
            elif clockwise:
                _lane_marking.id = 1 + lane_index
                _deviation = - _lane_marking.id * self._lane_width
            else:
                _lane_marking.id = 1 + self._lane_num_clockwise + lane_index
                _deviation = (1 + lane_index) * self._lane_width

            _lane_marking.type = Marker.LINE_STRIP
            _lane_marking.action = Marker.ADD
            _lane_marking.pose = convert_to_message(tf.transformations.translation_matrix((0,0,0)))
            _lane_marking.scale.x=0.3 # lane marking width

            _lane_marking.color.r=1.0
            _lane_marking.color.g=1.0
            if lane_index==-1:
                _lane_marking.color.b=0.0
            else:
                _lane_marking.color.b=1.0
            _lane_marking.color.a=1.0

            lm_init_position = (0, _deviation, 0)
            lm_init_point = geometry_msgs.msg.Point(*lm_init_position)

            curve_radius = self._radius_curve_track + _deviation

            lm_length = 2 * self._length_straight_track + 2 * math.pi * curve_radius
            lm_discrete_num = int(lm_length/s)+1
            print(lm_discrete_num)
            lm_landmark1 = int(self._length_straight_track/s)
            lm_landmark2 = int((self._length_straight_track + math.pi * curve_radius)/s)
            lm_landmark3 = int((2* self._length_straight_track + math.pi * curve_radius)/s)

            theta = s / curve_radius

            for i in range(lm_discrete_num-1):
                if i < lm_landmark1:
                    pos = (i * s, _deviation, 0)
                elif i < lm_landmark2:
                    pos = (self._length_straight_track + math.sin((i - lm_landmark1) * theta) \
                    * curve_radius, _deviation + math.cos((i - lm_landmark1) * theta) * curve_radius - curve_radius, 0)
                elif i < lm_landmark3:
                    pos = (self._length_straight_track -(i - lm_landmark2) * s, _deviation - 2 * curve_radius, 0)
                else:
                    pos = (-math.sin((i - lm_landmark3) * theta) \
                    * curve_radius, _deviation - math.cos((i - lm_landmark3) * theta) * curve_radius - curve_radius, 0)

                _lane_marking.points.append(geometry_msgs.msg.Point(*pos))

            _lane_marking.points.append(lm_init_point)

            return _lane_marking

        map_markers = MarkerArray()
        _center_lane = _make_lane_markings()
        map_markers.markers.append(_center_lane)

        for i in range(self._lane_num_clockwise):
            _clockwise_lane = _make_lane_markings(True, i)
            map_markers.markers.append(_clockwise_lane)

        for i in range(self._lane_num_counterclockwise):
            _counterclockwise_lane = _make_lane_markings(False, i)
            map_markers.markers.append(_counterclockwise_lane)

        self._map_markers = map_markers


if __name__ == '__main__':
    try:
        lane_map = SimpleTrack()
        rospy.sleep(10)
        lane_map.publish_map()
    except rospy.ROSInterruptException:
        pass
