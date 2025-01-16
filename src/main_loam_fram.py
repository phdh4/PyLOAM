import cv2
import open3d
import rosbag
import sensor_msgs

from data_loader import NPYLoader
from laser_odometry import Odometry
from laser_mapping import Mapper
import numpy as np


def loam_frame(bag_file, frame_num):
    bag = rosbag.Bag(bag_file)  # bag包读取
    # loam初始化
    odometry = Odometry(config=None)
    mapper = Mapper(config=None)
    mapper.STACK_NUM = 1

    count = 0
    trans_list = []
    points_world = np.zeros([1, 3])
    name = []
    for topic, msg, t in bag.read_messages(topics=['/benewakeHornX2_lidar_visualization_A']):
        if count % frame_num == 0: # 每frame_num帧重新建图
            if count != 0:
                # point_cloud = open3d.geometry.PointCloud()
                # point_cloud.points = open3d.utility.Vector3dVector(points_world)
                # open3d.io.write_point_cloud(name[-1] + '.pcd', point_cloud)
                points_world = np.zeros([1, 3])
            name.append(str(t.secs)+"."+str(t.nsecs))
            odometry.rot_w_curr = np.eye(3)
            odometry.trans_w_curr = np.zeros((3, 1))
            odometry.transform = np.array([0., 0., 0., 0., 0., 0.])
            odometry.frame_count = 0

            mapper.rot_wmap_wodom = np.eye(3)
            mapper.trans_wmap_wodom = np.zeros((3, 1))
            mapper.frame_count = 0
            mapper.rot_wodom_curr = np.eye(3)
            mapper.trans_wodom_curr = np.zeros((3, 1))
            mapper.rot_w_curr = np.eye(3)
            mapper.trans_w_curr = np.zeros((3, 1))
            mapper.transform = np.zeros(6)

        cloud = sensor_msgs.point_cloud2.read_points_list(msg)
        cloud = point_list_to_cloud(cloud)
        # 开始建图
        surf_pts, corner_pts, odom = odometry.grab_frame(cloud)
        trans = mapper.map_frame(odom, corner_pts, surf_pts)
        trans_list.append(trans)

        points_world = np.vstack((points_world, map_points_to_world(cloud[:, 0:3], trans)))
        count += 1

    return trans_list

def point_list_to_cloud(pts_list):
    cloud = []
    for pt in pts_list:
        pt_np = np.array([pt.x, pt.y, pt.z, pt.intensity, pt.timestamp_s, pt.timestamp_ns, pt.row, pt.channel])
        cloud.append(pt_np)
    cloud = np.vstack(cloud)
    ring = cloud[:, 6] * 4 + cloud[:, 7]
    ring = np.expand_dims(ring, axis=1)
    time_stamp = cloud[:, 4] + cloud[:, 5] * 1e-9
    rel_time = (time_stamp - time_stamp.min()) / (time_stamp.max() - time_stamp.min())
    rel_time = np.expand_dims(rel_time, axis=1)
    cloud = np.hstack((cloud[:, 0:4], ring))
    cloud = np.hstack((cloud, rel_time))
    return cloud

def get_rotation(roll, pitch, yaw):
    # 使用欧拉角（roll, pitch, yaw）计算旋转矩阵
    # 这里假设我们使用的是 ZYX 顺序（首先绕 Z 轴旋转，接着绕 Y 轴，最后绕 X 轴）
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # 按照 ZYX 顺序进行旋转
    R = np.matmul(R_z, np.matmul(R_y, R_x))
    return R

def map_points_to_world(points, transform):
    """
    将局部坐标系中的点云映射到世界坐标系
    :param points: 输入点云，形状为 (N, 3)，每行代表一个点的 (x, y, z) 坐标
    :return: 映射到世界坐标系的点云，形状为 (N, 3)
    """
    # 提取欧拉角（从self.transform获取），假设它们在self.transform的前3个位置
    roll, pitch, yaw = transform[0], transform[1], transform[2]

    # 获取旋转矩阵
    rotation_matrix = get_rotation(roll, pitch, yaw).T

    # 提取平移向量（从self.transform的后3个位置获取）
    translation = transform[3:].reshape(3, 1)

    # 将点云转换为 (N, 3) 的numpy数组
    points = np.array(points)

    # 将每个点应用旋转和平移，计算映射后的点
    points_world = np.dot(points, rotation_matrix.T) + translation.T

    return points_world

if __name__== '__main__':
    bag_file = "E:/hzy/data/depth_predict/6_2024-12-21-11-01-13_74.bag"
    loam_frame(bag_file, 1)
