import open3d
import rosbag
import sys, os
import sensor_msgs.point_cloud2
import numpy as np


def point_list_to_cloud(pts_list):
    cloud = []
    for pt in pts_list:
        pt_np = np.array([pt.x, pt.y, pt.z])
        cloud.append(pt_np)
    cloud = np.vstack(cloud)
    return cloud

if __name__=="__main__":
    bag = rosbag.Bag("E:/hzy/data/depth_predict/08/6_2024-12-19-21-06-25_389.bag")
    counter = 0
    for topic, msg, t in bag.read_messages(topics=['/benewakeHornX2_lidar_visualization_A']):
        cloud = sensor_msgs.point_cloud2.read_points_list(msg)
        time = msg.header.stamp# .to_sec()
        # print(time)
        cloud = point_list_to_cloud(cloud)
        sec = str(time.secs)
        nsec = str(time.nsecs)
        if len(nsec)!=9:
            print(nsec)
            nsec = nsec.zfill(9)
            print(nsec)
        name = sec + "." + nsec
        point_cloud = open3d.geometry.PointCloud()
        point_cloud.points = open3d.utility.Vector3dVector(cloud)

        open3d.io.write_point_cloud("E:/hzy/data/depth_predict/08/pcd/"+name + '.pcd', point_cloud)
