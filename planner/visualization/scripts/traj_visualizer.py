#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class TrajVisualizer:
    def __init__(self):
        rospy.init_node("traj_visualizer", anonymous=False)

        self.topic_name = rospy.get_param("~topic_name", "/record_traj_topic")
        self.frame_id = "map"
        self.num_robot = rospy.get_param("~num_robot", 4)

        self.line_width = rospy.get_param("~line_width", 0.15)
        self.point_size = rospy.get_param("~point_size", 0.18)
        self.arrow_scale = rospy.get_param("~arrow_scale", 0.35)
        self.min_dist_append = rospy.get_param("~min_dist_append", 0.02)
        self.show_heading = rospy.get_param("~show_heading", True)
        self.z_offset = rospy.get_param("~z_offset", 0.05)

        self.pub = rospy.Publisher("/traj_markers", MarkerArray, queue_size=10)
        self.sub = rospy.Subscriber(self.topic_name, Float64MultiArray, self.cb, queue_size=50)

        self.trajs = [[] for _ in range(self.num_robot)]
        self.last_theta = [0.0 for _ in range(self.num_robot)]

        # 4个机器人颜色
        self.colors = [
            (1.0, 0.0, 0.0, 1.0),   # 红
            (0.0, 1.0, 0.0, 1.0),   # 绿
            (0.0, 0.4, 1.0, 1.0),   # 蓝
            (1.0, 0.8, 0.0, 1.0),   # 黄
        ]

        rospy.loginfo("traj_visualizer started")
        rospy.loginfo("subscribe topic: %s", self.topic_name)
        rospy.loginfo("publish markers: /traj_markers")
        # rospy.loginfo("frame_id: %s", self.frame_id)

    def make_point(self, x, y, z=0.0):
        p = Point()
        p.x = x
        p.y = y
        p.z = z
        return p

    def append_if_needed(self, rid, x, y):
        if len(self.trajs[rid]) == 0:
            self.trajs[rid].append((x, y))
            return

        lx, ly = self.trajs[rid][-1]
        d2 = (x - lx) ** 2 + (y - ly) ** 2
        if d2 >= self.min_dist_append ** 2:
            self.trajs[rid].append((x, y))

    def build_line_marker(self, rid, marker_id, stamp):
        color = self.colors[rid % len(self.colors)]

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = stamp
        m.ns = "traj_line"
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.3
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = color[3]
        m.lifetime = rospy.Duration(0)

        for (x, y) in self.trajs[rid]:
            m.points.append(self.make_point(x, y, self.z_offset))

        return m

    def build_head_marker(self, rid, marker_id, stamp):
        color = self.colors[rid % len(self.colors)]
        x, y = self.trajs[rid][-1]

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = stamp
        m.ns = "traj_head"
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = self.z_offset
        m.pose.orientation.w = 1.0
        m.scale.x = self.point_size
        m.scale.y = self.point_size
        m.scale.z = self.point_size
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0)
        return m

    def build_arrow_marker(self, rid, marker_id, stamp):
        color = self.colors[rid % len(self.colors)]
        x, y = self.trajs[rid][-1]
        theta = self.last_theta[rid]

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = stamp
        m.ns = "traj_heading"
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.06   # shaft diameter
        m.scale.y = 0.12   # head diameter
        m.scale.z = 0.12   # head length
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0)

        p_start = self.make_point(x, y, self.z_offset + 0.02)
        p_end = self.make_point(
            x + self.arrow_scale * math.cos(theta),
            y + self.arrow_scale * math.sin(theta),
            self.z_offset + 0.02
        )
        m.points = [p_start, p_end]
        return m

    def cb(self, msg):
        required_len = self.num_robot * 5
        if len(msg.data) < required_len:
            rospy.logwarn_throttle(
                1.0,
                "record_traj_topic length=%d, but expected at least %d",
                len(msg.data), required_len
            )
            return

        for r in range(self.num_robot):
            x = msg.data[r * 5 + 0]
            y = msg.data[r * 5 + 1]
            theta = msg.data[r * 5 + 2]
            self.last_theta[r] = theta
            self.append_if_needed(r, x, y)

        marker_array = MarkerArray()
        now = rospy.Time.now()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        mid = 0
        for r in range(self.num_robot):
            if len(self.trajs[r]) == 0:
                continue

            marker_array.markers.append(self.build_line_marker(r, mid, now))
            mid += 1

            marker_array.markers.append(self.build_head_marker(r, mid, now))
            mid += 1

            if self.show_heading:
                marker_array.markers.append(self.build_arrow_marker(r, mid, now))
                mid += 1

        self.pub.publish(marker_array)


if __name__ == "__main__":
    try:
        TrajVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass