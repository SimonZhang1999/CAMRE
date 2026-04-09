#!/usr/bin/env python3

# jplaced@unizar.es
# 2022, Universidad de Zaragoza

# The filter nodes receives the detected frontier points from all the detectors,
# filters the points, and passes them to the assigner node to command the robots.
# Filtration includes the delection of old and invalid points, and it also
# discards redundant points.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy
import tf
import sys
import time

import numpy as np
from sklearn.cluster import MeanShift
from copy import copy, deepcopy

sys.path.insert(0, "/home/weijian/LoS_constrained_navigation/catkin_ws_connectivity/src/multi_slam_realm/scripts")



from constants import USE_GPU_
from functions import gridValue, createMarker, gridNeighborHasObstacle

if USE_GPU_:
    from functions import informationGain_NUMBA
else:
    from functions import informationGain

from multi_slam_realm.msg import PointArray

from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PointStamped
from dynamic_reconfigure.server import Server
from multi_slam_realm.cfg import informationGainConfig

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
map_data_ = OccupancyGrid()
frontiers_ = []
f_timestamps_ = []
global_map_ = []
INFORMATION_THRESHOLD_ = 0.35


def reconfigureCallback(config, level):
    global INFORMATION_THRESHOLD_
    rospy.logwarn("""Reconfigure Request! InfoGain threshold changed to: {ig_threshold}""".format(**config))
    INFORMATION_THRESHOLD_ = config["ig_threshold"]
    return config


def frontiersCallBack(data, args):  
    """ Get detected frontier points from detector, like openCV
    args[0]: tfListener
    args[1]: global_frame, which is map
    """
    global frontiers_, f_timestamps_
    # tfLisn = args[0]
    # target_frame = args[1]

    # data.header.stamp = rospy.Time(0)   # 用最新可用的 TF
    # transformedPoint = tfLisn.transformPoint(target_frame, data)
    transformedPoint = args[0].transformPoint(args[1], data)
    x = np.array([transformedPoint.point.x, transformedPoint.point.y])
    x_t = data.header.stamp.to_sec()

    # Only add if not already there
    # Use temp variables to avoid errors due to global variables dimension (multiple ROS cb same time)
    temp_new_frontier = []
    temp_new_frontier = copy(x.tolist())
    temp_previous_frontiers = []
    temp_previous_frontiers = np.asarray(copy(frontiers_)).tolist()
    temp_previous_times = []
    temp_previous_times = copy(f_timestamps_)

    if len(temp_previous_frontiers) == len(temp_previous_times):
        assert (len(temp_previous_frontiers) == len(temp_previous_times))
        if temp_new_frontier in temp_previous_frontiers:
            repeated_idx = temp_previous_frontiers.index(temp_new_frontier)
            temp_previous_times[repeated_idx] = x_t
        else:  # Otherwise, update timestamp
            temp_previous_frontiers.append(x)
            temp_previous_times.append(x_t)

        # Delete too old points
        originalLen = len(temp_previous_frontiers)
        curr_time = rospy.get_time()
        assert originalLen == len(temp_previous_times)

        updated_frontiers = []
        updated_timestamps = []
        for i in range(0, originalLen):
            t_diff = np.abs(temp_previous_times[i] - curr_time)
            if t_diff > 5.0:
                rospy.logdebug("Deleted a frontier with timestamp diff = " + str(t_diff))
            else:
                updated_frontiers.append(temp_previous_frontiers[i])
                updated_timestamps.append(temp_previous_times[i])

        frontiers_ = copy(updated_frontiers)
        f_timestamps_ = copy(updated_timestamps)
        assert (len(frontiers_) == len(f_timestamps_))
    else:
        rospy.logerr("Frontier callback failed due to dimension mismatch of " + str(
            len(temp_previous_frontiers) - len(temp_previous_times)) + ". Skipping callback.")


def mapCallBack(data):
    """ callback for occupancy map """
    global map_data_
    map_data_ = data


def globalMapCallback(data):
    """ callback for cost map topic"""
    global global_map_
    global_map_ = data


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node():
    global frontiers_, map_data_, global_map_
    rospy.init_node('filter', anonymous=False)

    # Fetch all parameters
    ns = rospy.get_namespace()
    map_topic = rospy.get_param('/map', '/map')
    threshold = rospy.get_param('costmap_clearing_threshold', 70)
    goals_topic = rospy.get_param('/goals_topic', ns + 'detected_points')
    rateHz = rospy.get_param('rate', 20)
    robot_frame = rospy.get_param('/robot_frame', ns + 'base_link')

    srv = Server(informationGainConfig, reconfigureCallback)

    # print(map_topic)
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)

    # global_map_ = OccupancyGrid()

    # rospy.Subscriber(global_costmap_topic, OccupancyGrid, globalMapCallback)

    # Wait if map map has not been received yet
    while len(map_data_.data) < 1:
        rospy.loginfo(rospy.get_name() + ': Filter is waiting for the map.')
        rospy.sleep(0.5)
        pass

    # # Wait if global costmap map has not been received yet
    # while len(global_map_.data) < 1:
    #     rospy.loginfo(rospy.get_name() + ': Filter is waiting for the global costmap.')
    #     rospy.sleep(0.5)
    #     pass

    rospy.loginfo("Filter received local and global costmaps.")

    global_frame = map_data_.header.frame_id
    tfLisn = tf.TransformListener()
    tfLisn.waitForTransform(global_frame, robot_frame, rospy.Time(0), rospy.Duration(10.0))

    rospy.Subscriber(goals_topic, PointStamped, callback=frontiersCallBack, callback_args=[tfLisn, global_frame])

    # Publishers
    pub_raw_frontiers = rospy.Publisher('raw_frontiers_marker', Marker, queue_size=10)
    pub_centroids = rospy.Publisher('centroids_marker', Marker, queue_size=10)
    pub_centroids_points = rospy.Publisher('centroids_points', PointArray, queue_size=10)

    # Wait if no frontier is received yet
    counter = 0
    while len(frontiers_) < 1:
        if counter == 0:
            rospy.loginfo("Filter is waiting for frontiers.")
            counter = 1

    rospy.loginfo("Filter received frontiers.")

    points = createMarker(frame=map_data_.header.frame_id, ns="raw_frontiers", colors=[255, 255, 0.0], scale=0.2)
    points_clust = createMarker(frame=map_data_.header.frame_id, ns="filtered_frontiers", colors=[0.0, 255, 0.0])

    p = Point()
    p.z = 0

    tempPointStamped = PointStamped()
    tempPointStamped.header.frame_id = map_data_.header.frame_id
    tempPointStamped.header.stamp = rospy.Time(0)
    tempPointStamped.point.z = 0.0

    tempPointArray = PointArray()
    tempPoint = Point()
    tempPoint.z = 0.0

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    rate = rospy.Rate(rateHz)
    while not rospy.is_shutdown():
        t_loop_start = time.perf_counter()

        delete_obstacle = 0
        delete_ig = 0

        # Clustering frontier points
        centroids = []
        front = deepcopy(frontiers_)

        if len(front) > 1:
            ms = MeanShift(bandwidth=2)
            ms.fit(front)
            centroids = ms.cluster_centers_
        elif len(front) == 1:  # If there is only one frontier no need for clustering
            centroids = front

        # Clearing frontiers
        originalCentroidsLen = len(centroids)
        for zp in range(0, originalCentroidsLen):
            z = zp - originalCentroidsLen + len(centroids)

            cond = False
            tempPointStamped.point.x = centroids[z][0]
            tempPointStamped.point.y = centroids[z][1]

            transformedPoint = tfLisn.transformPoint(map_data_.header.frame_id, tempPointStamped)
            x = np.array([transformedPoint.point.x, transformedPoint.point.y])
            cond = (gridValue(map_data_, x) >= threshold) or cond
            if gridNeighborHasObstacle(map_data_, x, 1):
                cond = True

            if USE_GPU_:
                ig = informationGain_NUMBA(map_data_.info.resolution, map_data_.info.width,
                                        map_data_.info.origin.position.x,
                                        map_data_.info.origin.position.y, np.array(map_data_.data), centroids[z][0],
                                        centroids[z][1], 1.0)
            else:
                ig = informationGain(map_data_, [centroids[z][0], centroids[z][1]], 1.0)

            if cond:
                centroids = np.delete(centroids, z, axis=0)
                rospy.logdebug("Deleted a frontier with information gain = " + str(ig))

        rospy.logdebug("Frontier centroids len=" + str(len(centroids)) + ", frontiers len=" + str(len(front)))

        # Publishing
        tempPointArray.points = []
        pp = []
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            tempPointArray.points.append(copy(tempPoint))
            pp.append(copy(tempPoint))

        if ns[-2] == "1":
            points_clust.color.r = 1.0
            points_clust.color.g = 0.0
            points_clust.color.b = 0.0
        if ns[-2] == "2":
            points_clust.color.r = 0.0
            points_clust.color.g = 1.0
            points_clust.color.b = 0.0
        if ns[-2] == "3":
            points_clust.color.r = 0.0
            points_clust.color.g = 0.0
            points_clust.color.b = 1.0

        points_clust.id += 1
        points_clust.points = pp
        pub_centroids.publish(points_clust)
        pub_centroids_points.publish(tempPointArray)

        pp = []
        for q in front:
            p.x = q[0]
            p.y = q[1]
            pp.append(copy(p))
        points.id += 1
        points.points = pp
        pub_raw_frontiers.publish(points)

        t_loop_end = time.perf_counter()
        loop_ms = (t_loop_end - t_loop_start) * 1000.0

        rospy.loginfo_throttle(
            1.0,
            "filter loop time = %.3f ms, raw_frontiers = %d, centroids = %d"
            % (loop_ms, len(front), len(centroids))
        )

        rate.sleep()


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
