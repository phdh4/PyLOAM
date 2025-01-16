import glob

import cv2
import numpy as np
import open3d
from typing_extensions import overload


class VisualizeProjection:
    def __init__(self, params_path: str):
        self.K, self.D, self.rvec, self.tvec = self.parse_params(params_path)


    def parse_params(self, param_path:str):
        param = cv2.FileStorage(param_path, cv2.FILE_STORAGE_READ)
        K = param.getNode("K").mat()
        D = param.getNode("D").mat()
        R = param.getNode("R").mat()
        T = param.getNode("T").mat()
        rvec, _ = cv2.Rodrigues(R)
        tvec = T
        return K, D, rvec, tvec


    def get_color(self, cur_depth, max_depth, min_depth):
        scale = (max_depth - min_depth) / 10
        if cur_depth < min_depth:
            return (0, 0, 255)  # 返回蓝色
        elif cur_depth < min_depth + scale:
            green = int((cur_depth - min_depth) / scale * 255)
            return (0, green, 255)  # 返回蓝到黄的渐变色
        elif cur_depth < min_depth + scale * 2:
            red = int((cur_depth - min_depth - scale) / scale * 255)
            return (0, 255, 255 - red)  # 返回黄到红的渐变色
        elif cur_depth < min_depth + scale * 4:
            blue = int((cur_depth - min_depth - scale * 2) / scale * 255)
            return (blue, 255, 0)  # 返回红到绿的渐变色
        elif cur_depth < min_depth + scale * 7:
            green = int((cur_depth - min_depth - scale * 4) / scale * 255)
            return (255, 255 - green, 0)  # 返回绿到黄的渐变色
        elif cur_depth < min_depth + scale * 10:
            blue = int((cur_depth - min_depth - scale * 7) / scale * 255)
            return (255, 0, blue)  # 返回黄到蓝的渐变色
        else:
            return (255, 0, 255)  # 返回紫色


    def projection(self, img_file_list, point_file_list, show: bool = False):
        length = len(img_file_list)
        for i in range(length):
            if i >= 0:
                print(img_file_list[i], point_file_list[i])
                image_origin = cv2.imread(img_file_list[i])
                pcd = open3d.io.read_point_cloud(point_file_list[i])
                pts_3d = np.asarray(pcd.points)
                max_depth = np.max(pts_3d, axis=0)[0]
                min_depth = np.min(pts_3d, axis=0)[0]
                pts_2d, _ = cv2.projectPoints(np.array(pts_3d), self.rvec, self.tvec, self.K, self.D)
                pts_2d = pts_2d.squeeze()
                image_project = image_origin.copy()
                for i, point_2d in enumerate(pts_2d):
                    x, y = int(point_2d[0]), int(point_2d[1])
                    if 0 <= x < image_origin.shape[1] and 0 <= y < image_origin.shape[0]:
                        cur_depth = pts_3d[i][0]
                        color = self.get_color(cur_depth, max_depth, min_depth)  # 根据深度获取颜色
                        image_project[y-1:y+1, x-1:x+1] = color  # 设置点云的颜色
                if show:
                    # cv2.imshow("origin image", image_origin)
                    cv2.imshow("project image", image_project)
                    cv2.waitKey(4)

if __name__ == "__main__":
    a = VisualizeProjection('./T08-TC2-lidar2camera-long.yml')
    img_file_list = glob.glob("E:/hzy/data/depth_predict/image/"+"*.png")
    point_file_list = glob.glob("E:/hzy/data/depth_predict/pcd/"+"*.pcd")
    a.projection(img_file_list, point_file_list, True)
