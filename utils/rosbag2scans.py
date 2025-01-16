import rosbag
import sys, os
import sensor_msgs.point_cloud2
import numpy as np


def point_list_to_cloud(pts_list):
    cloud = []
    for pt in pts_list:
        pt_np = np.array([pt.x, pt.y, pt.z, pt.intensity, pt.timestamp_s, pt.timestamp_ns, pt.row, pt.channel])
        cloud.append(pt_np)
    cloud = np.vstack(cloud)
    return cloud

if __name__=="__main__":
    bag = rosbag.Bag("E:/hzy/data/depth_predict/6_2024-12-21-11-01-13_74.bag")
    counter = 0
    for topic, msg, t in bag.read_messages(topics=['/benewakeHornX2_lidar_visualization_A']):
        cloud = sensor_msgs.point_cloud2.read_points_list(msg)
        cloud = point_list_to_cloud(cloud)
        np.save(os.path.join("./", str(counter) + '.npy'), cloud)
        counter += 1

