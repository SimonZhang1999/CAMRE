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


class RecordLosMinPlot:
    def __init__(self):
        rospy.init_node("record_los_min_plot", anonymous=False)

        self.topic_name = rospy.get_param("~topic_name", "/record_graph_edges")
        self.output_dir = rospy.get_param(
            "~output_dir",
            os.path.join(os.path.expanduser("~"), ".ros", "los_record")
        )
        self.file_prefix = rospy.get_param("~file_prefix", "los_min")
        self.use_sim_time_axis = rospy.get_param("~use_sim_time_axis", True)

        os.makedirs(self.output_dir, exist_ok=True)

        self.times = []
        self.min_los_values = []
        self.edge_counts = []

        self.last_time = None

        self.sub = rospy.Subscriber(
            self.topic_name,
            Float64MultiArray,
            self.cb,
            queue_size=200
        )

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("record_los_min_plot started")
        rospy.loginfo("subscribe topic: %s", self.topic_name)
        rospy.loginfo("output_dir: %s", self.output_dir)

    def cb(self, msg):
        data = list(msg.data)

        if len(data) == 0:
            rospy.logwarn_throttle(1.0, "Received empty /record_graph_edges message")
            return

        if len(data) % 3 != 0:
            rospy.logwarn_throttle(
                1.0,
                "record_graph_edges length=%d is not divisible by 3, ignoring malformed tail",
                len(data)
            )

        n_triplets = len(data) // 3
        if n_triplets == 0:
            return

        if self.use_sim_time_axis:
            t = rospy.Time.now().to_sec()
        else:
            t = rospy.get_time()

        if self.last_time is not None and abs(t - self.last_time) < 1e-9:
            return
        self.last_time = t

        los_vals = []
        for k in range(n_triplets):
            base = 3 * k
            # i = int(round(data[base + 0]))
            # j = int(round(data[base + 1]))
            los_min = float(data[base + 2])
            los_vals.append(los_min)

        if len(los_vals) == 0:
            return

        frame_min_los = min(los_vals)

        self.times.append(t)
        self.min_los_values.append(frame_min_los)
        self.edge_counts.append(n_triplets)

    def save_csv(self):
        if len(self.times) == 0:
            rospy.logwarn("No los_min data recorded, skip csv save.")
            return None

        csv_path = os.path.join(self.output_dir, f"{self.file_prefix}.csv")

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "min_los", "num_edges"])

            t0 = self.times[0]
            for t, los, n in zip(self.times, self.min_los_values, self.edge_counts):
                writer.writerow([t - t0, los, n])

        rospy.loginfo("Saved csv to: %s", csv_path)
        return csv_path

    def plot_curve(self):
        if len(self.times) == 0:
            rospy.logwarn("No los_min data recorded, skip plotting.")
            return None

        times = np.array(self.times, dtype=float)
        times = times - times[0]
        vals = np.array(self.min_los_values, dtype=float)

        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(times, vals, linewidth=2.0, label="minimum los_min per frame")
        ax.scatter(times, vals, s=12)

        ax.set_title("Minimum LoS Value per Frame")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Minimum los_min")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend()

        fig.tight_layout()

        png_path = os.path.join(self.output_dir, f"{self.file_prefix}.png")
        fig.savefig(png_path, dpi=200)
        plt.close(fig)

        rospy.loginfo("Saved plot to: %s", png_path)
        return png_path

    def on_shutdown(self):
        rospy.loginfo("Shutting down record_los_min_plot, saving data...")
        self.save_csv()
        self.plot_curve()


if __name__ == "__main__":
    try:
        node = RecordLosMinPlot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass