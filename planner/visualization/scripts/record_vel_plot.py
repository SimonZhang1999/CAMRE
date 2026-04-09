#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import rospy
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from std_msgs.msg import Float64MultiArray


class RecordSpeedPlot:
    def __init__(self):
        rospy.init_node("record_speed_plot", anonymous=False)

        self.topic_name = rospy.get_param("~topic_name", "/record_traj_topic")
        self.num_robot = rospy.get_param("~num_robot", 4)
        self.block_size = rospy.get_param("~block_size", 5)

        # 每个机器人 block 里的线速度索引，默认取第4个数，即 r*5+3
        self.speed_index_in_block = rospy.get_param("~speed_index_in_block", 3)

        # 最后一个元素是否是 leader index
        self.leader_index_pos = rospy.get_param("~leader_index_pos", self.num_robot * self.block_size)

        # 输出目录
        default_out_dir = os.path.join(os.path.expanduser("~"), ".ros", "speed_record")
        self.output_dir = rospy.get_param("~output_dir", default_out_dir)

        # 文件名前缀
        self.file_prefix = rospy.get_param("~file_prefix", "robot_speed")

        # 去重：只有时间前进才记录，防止重复
        self.last_time = None

        # 数据缓存
        self.times = []
        self.leader_indices = []
        self.speeds = []   # list of [v0, v1, ..., vN-1]

        os.makedirs(self.output_dir, exist_ok=True)

        self.sub = rospy.Subscriber(self.topic_name, Float64MultiArray, self.cb, queue_size=200)

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("record_speed_plot started")
        rospy.loginfo("subscribe topic: %s", self.topic_name)
        rospy.loginfo("num_robot: %d", self.num_robot)
        rospy.loginfo("block_size: %d", self.block_size)
        rospy.loginfo("speed_index_in_block: %d", self.speed_index_in_block)
        rospy.loginfo("leader_index_pos: %d", self.leader_index_pos)
        rospy.loginfo("output_dir: %s", self.output_dir)

    def cb(self, msg):
        required_len = self.num_robot * self.block_size + 1
        if len(msg.data) < required_len:
            rospy.logwarn_throttle(
                1.0,
                "record_traj_topic length=%d < required=%d",
                len(msg.data), required_len
            )
            return

        t = rospy.Time.now().to_sec()

        # 避免重复时间戳
        if self.last_time is not None and abs(t - self.last_time) < 1e-9:
            return
        self.last_time = t

        try:
            leader_idx = int(round(msg.data[self.leader_index_pos]))
        except Exception:
            leader_idx = -1

        v_list = []
        for r in range(self.num_robot):
            idx = r * self.block_size + self.speed_index_in_block
            v = float(msg.data[idx])
            v_list.append(v)

        self.times.append(t)
        self.leader_indices.append(leader_idx)
        self.speeds.append(v_list)

    def save_csv(self):
        if len(self.times) == 0:
            rospy.logwarn("No speed data recorded, skip csv save.")
            return None

        csv_path = os.path.join(self.output_dir, f"{self.file_prefix}.csv")

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)

            header = ["time", "leader_idx"]
            for r in range(self.num_robot):
                header.append(f"robot_{r}_speed")
            writer.writerow(header)

            for i in range(len(self.times)):
                row = [self.times[i], self.leader_indices[i]] + self.speeds[i]
                writer.writerow(row)

        rospy.loginfo("Saved csv to: %s", csv_path)
        return csv_path

    def plot_speed(self):
        if len(self.times) == 0:
            rospy.logwarn("No speed data recorded, skip plotting.")
            return None

        times = np.array(self.times, dtype=float)
        times = times - times[0]   # 从0开始更好看
        speeds = np.array(self.speeds, dtype=float)  # shape: [T, N]
        leaders = np.array(self.leader_indices, dtype=int)

        fig, ax = plt.subplots(figsize=(12, 6))

        leader_color = "red"
        follower_color = "blue"

        for r in range(self.num_robot):
            vr = speeds[:, r]

            # 先画一条淡灰色底线，帮助看连续趋势
            ax.plot(times, vr, linestyle="-", linewidth=1.0, alpha=0.35, color="gray")

            # leader 时刻
            leader_mask = (leaders == r)

            # 非 leader 时刻
            follower_mask = (leaders != r)

            # leader 点
            if np.any(leader_mask):
                ax.scatter(
                    times[leader_mask],
                    vr[leader_mask],
                    s=10,
                    c=leader_color,
                    label="leader speed" if r == 0 else None
                )

            # follower 点
            if np.any(follower_mask):
                ax.scatter(
                    times[follower_mask],
                    vr[follower_mask],
                    s=10,
                    c=follower_color,
                    label="follower speed" if r == 0 else None
                )

        ax.set_title("All Robot Linear Speeds")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Linear Speed")
        ax.grid(True, linestyle="--", alpha=0.4)

        handles, labels = ax.get_legend_handles_labels()
        uniq = {}
        for h, l in zip(handles, labels):
            if l not in uniq and l != "":
                uniq[l] = h
        ax.legend(uniq.values(), uniq.keys())

        fig.tight_layout()

        png_path = os.path.join(self.output_dir, f"{self.file_prefix}.png")
        fig.savefig(png_path, dpi=200)
        plt.close(fig)

        rospy.loginfo("Saved plot to: %s", png_path)
        return png_path

    def on_shutdown(self):
        rospy.loginfo("Shutting down record_speed_plot, saving data...")
        self.save_csv()
        self.plot_speed()


if __name__ == "__main__":
    try:
        node = RecordSpeedPlot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass