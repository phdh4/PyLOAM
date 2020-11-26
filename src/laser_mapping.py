import numpy as np
import open3d as o3d
from utils import *

class Mapper:
    def __init__(self, config=None):
        self.rot_wmap_wodom = np.eye(3)
        self.trans_wmap_wodom = np.zeros((3,1))
        self.frame_count = 0
        # Parameters
        self.CLOUD_WIDTH = 21
        self.CLOUD_HEIGHT = 21
        self.CLOUD_DEPTH = 11
        self.CORNER_VOXEL_SIZE = 0.2
        self.SURF_VOXEL_SIZE = 0.4
        self.MAP_VOXEL_SIZE = 0.6
        self.CUBE_NUM = self.CLOUD_DEPTH * self.CLOUD_HEIGHT * self.CLOUD_WIDTH

        self.cloud_center_width = int(self.CLOUD_WIDTH/2)
        self.cloud_center_height = int(self.CLOUD_HEIGHT/2)
        self.cloud_center_depth = int(self.CLOUD_DEPTH/2)
        self.valid_index = [-1] * 125
        self.surround_index = [-1] * 125

        self.cloud_corner_array = [None] * self.CUBE_NUM
        self.cloud_surf_array = [None] * self.CUBE_NUM

        self.rot_wodom_curr = np.eye(3)
        self.trans_wodom_curr = np.zeros(3,1)
        self.rot_wmap_wodom = np.eye(3)
        self.trans_wmap_wodom = np.eye(3)
        self.rot_w_curr = np.eye(3)
        self.trans_w_curr = np.zeros(3,1)
        self.transform = np.zeros(6)

    def transform_associate_to_map(self, rot_wodom_curr, trans_wodom_curr):
        rot_w_curr = np.matmul(self.rot_wmap_wodom, rot_wodom_curr)
        trans_w_curr = np.matmul(self.rot_wmap_wodom, trans_wodom_curr) + self.trans_wmap_wodom
        # TODO: Update self.transform
        return rot_w_curr, trans_w_curr

    def point_associate_to_map(self, pt):
        pt_out = np.matmul(self.rot_w_curr, pt.reshape(3, 1)) + self.trans_w_curr.reshape(3, 1)
        return pt_out
    
    def transform_update(self):
        # TODO: Update transforms

    def map_frame(self, odom, corner_last, surf_last):
        rot_w_curr, trans_w_curr = self.transform_associate_to_map(rot_wodom_curr, trans_wodom_curr)

        cube_center_i = int((trans_w_curr[0] + 25.0) / 50.0) + self.cloud_center_width
        cube_center_j = int((trans_w_curr[1] + 25.0) / 50.0) + self.cloud_center_height
        cube_center_k = int((trans_w_curr[2] + 25.0) / 50.0) + self.cloud_center_depth

        if trans_w_curr[0] + 25.0 < 0:
            cube_center_i -= 1
        if trans_w_curr[1] + 25.0 < 0:
            cube_center_j -= 1
        if trans_w_curr[2] + 25.0 < 0:
            cube_center_k -= 1
        
        while cube_center_i < 3:
            for j in range(self.CLOUD_HEIGHT):
                for k in range(self.CLOUD_DEPTH):
                    pass
                    # TODO: Move the cube
            
            cube_center_i += 1
            self.cloud_center_width += 1
        
        while cube_center_i >= self.CLOUD_WIDTH - 3:
            for j in range(self.CLOUD_HEIGHT):
                for k in range(self.CLOUD_DEPTH):
                    pass
            
            cube_center_i -= 1
            self.cloud_center_width -= 1

        while cube_center_j < 3:
            for i in range(self.CLOUD_WIDTH):
                for k in range(self.CLOUD_DEPTH):
                    pass

            cube_center_j += 1
            self.cloud_center_height += 1
        
        while cube_center_j >= self.CLOUD_HEIGHT - 3:
            for i in range(self.CLOUD_WIDTH):
                for k in range(self.CLOUD_DEPTH):
                    pass
            
            cube_center_j -= 1
            self.cloud_center_height -= 1

        while cube_center_k < 3:
            for i in range(self.CLOUD_WIDTH):
                for j in range(self.CLOUD_HEIGHT):
                    pass
            
            cube_center_k += 1
            self.cloud_center_depth += 1
        
        while cube_center_k >= self.CLOUD_DEPTH - 3:
            for i in range(self.CLOUD_WIDTH):
                for j in range(self.CLOUD_HEIGHT):
                    pass
            
            cube_center_k -= 1
            self.cloud_center_depth -= 1
        
        valid_cloud_num = 0
        surround_cloud_num = 0

        for i in range(cube_center_i - 2, cube_center_i + 3):
            for j in range(cube_center_j - 2, cube_center_j + 3):
                for k in range(cube_center_k - 1, cube_center_k + 2):
                    if i>=0 and i<self.CLOUD_WIDTH and j>=0 and j<self.CLOUD_HEIGHT and k>=0 and k<self.CLOUD_DEPTH:
                        self.valid_index[valid_cloud_num] = i + j * self.CLOUD_WIDTH + k * self.CLOUD_HEIGHT * self.CLOUD_WIDTH
                        valid_cloud_num += 1
                        self.surround_index[surround_index] = i + j * self.CLOUD_WIDTH + k * self.CLOUD_HEIGHT * self.CLOUD_WIDTH
                        surround_index += 1
        
        map_corner_list = []
        map_surf_list = []
        for i in range(valid_cloud_num):
            map_corner_list.append(self.cloud_corner_array[self.valid_index[i]])
            map_surf_list.append(self.cloud_surf_array[self.valid_index[i]])
        
        if len(map_corner_list) > 0:
            corner_from_map = np.vstack(map_corner_list)

        if len(map_surf_list) > 0:
            surf_from_map = np.vstack(map_surf_list)
        
        _, corner_last_ds = downsample_filter(corner_last, self.CORNER_VOXEL_SIZE)
        _, surf_last_ds = downsample_filter(surf_last, self.SURF_VOXEL_SIZE)

        if len(map_corner_list) > 0 and len(map_surf_list) > 0 and corner_from_map.shape[0] > 10 and surf_from_map.shape[0] > 100:
            corner_map_tree = o3d.geometry.KDTreeFlann(np.transpose(corner_from_map[:, :3]))
            surf_map_tree = o3d.geometry.KDTreeFlann(np.transpose(surf_from_map[:, :3]))

            for iter_num in range(10):
                coeff_list = []
                pt_list = []
                
                # Find corner correspondences
                for i in range(corner_last_ds.shape[0]):
                    point_sel = self.point_associate_to_map(corner_last_ds[i, :3])
                    [_, ind, dist] = corner_map_tree.search_knn_vector_3d(point_sel, 5)

                    if dist[4] < 1.0:
                        mean, cov = get_mean_cov(corner_from_map[ind, :3])
                        vals, vecs = np.linalg.eig(cov)
                        idx = vals.argsort() # Sort ascending
                        vals = vals[idx]
                        vecs = vecs[:, idx]

                        if vals[2] > 3 * vals[1]:
                            point_a = center + 0.1 * vecs[:, 2]
                            point_b = center - 0.1 * vecs[:, 2]

                            edge_normal = np.cross((point_sel - point_a), (point_sel - point_b), axis=0)
                            edge_norm = np.linalg.norm(edge_normal)
                            ab = point_a - point_b
                            ab_norm = np.linalg.norm(ab)

                            la = (ab[1]*edge_normal[2] + ab[2]*edge_normal[1]) / (ab_norm*edge_norm)
                            lb = -(ab[0]*edge_normal[2] - ab[2]*edge_normal[0]) / (ab_norm*edge_norm)
                            lc = -(ab[0]*edge_normal[1] + ab[1]*edge_normal[0]) / (ab_norm*edge_norm)

                            ld = edge_norm / ab_norm

                            s = 1 - 0.9 * np.abs(ld)

                            if s > 0.1:
                                pt_list.append(corner_last_ds[i, :3])
                                coeff_list.append(np.array([s*la, s*lb, s*lc, s*ld]))

                # Find surface correspondences
                for i in range(surf_last_ds.shape[0]):
                    point_sel = self.point_associate_to_map(surf_last_ds[i, :3])
                    [_, ind, dist] = surf_map_tree.search_knn_vector_3d(point_sel, 5)

                    if dist[4] < 1.0:
                        surf_normal = np.linalg.solve(surf_from_map[ind, :3], -np.ones(5))
                        surf_norm = np.linalg.norm(surf_normal)
                        coeff = np.append(surf_normal, 1.0) / surf_norm

                        surf_homo = np.concatenate((surf_from_map[ind, :3], np.ones(5,1)), axis=1)
                        plane_residual = np.abs(np.matmul(surf_homo, coeff.reshape(4,1)))

                        if np.any(plane_residual > 0.2):
                            continue
                        
                        pd2 = np.dot(np.append(point_sel, 1), coeff)
                        s = 1 - 0.9 * np.abs(pd2) / np.sqrt(np.linalg.norm(point_sel))

                        coeff[3] = pd2
                        coeff = s*coeff

                        if s > 0.1:
                            coeff_list.append(coeff)
                            pt_list.append(surf_last_ds[i, :3])
                
                if len(coeff_list) < 50:
                    continue

                srx = np.sin(self.transform[0])
                crx = np.cos(self.transform[0])
                sry = np.sin(self.transform[1])
                cry = np.cos(self.transform[1])
                srz = np.sin(self.transform[2])
                crz = np.cos(self.transform[2])

                A_mat = []
                B_mat = []

                for i in range(len(coeff_list)):
                    A_tmp = np.zeros(1,6)
                    B_tmp = np.zeros(1,1)
                    A_tmp[0, 0] = (crx*sry*srz*pt_list[i][0] + crx*crz*sry*pt_list[i][1] - srx*sry*pt_list[i][2]) * coeff_list[i][0]
                        + (-srx*srz*pt_list[i][0] - crz*srx*pt_list[i][1] - crx*pt_list[i][2]) * coeff_list[i][1]
                        + (crx*cry*srz*pt_list[i][0] + crx*cry*crz*pt_list[i][1] - cry*srx*pt_list[i][2]) * coeff_list[i][2]

                    A_tmp[0, 1] = ((cry*srx*srz - crz*sry)*pt_list[i][0] 
                        + (sry*srz + cry*crz*srx)*pt_list[i][1] + crx*cry*pt_list[i][2]) * coeff_list[i][0]
                        + ((-cry*crz - srx*sry*srz)*pt_list[i][0]
                        + (cry*srz - crz*srx*sry)*pt_list[i][1] - crx*sry*pt_list[i][2]) * coeff_list[i][2]

                    A_tmp[0, 2] = ((crz*srx*sry - cry*srz)*pt_list[i][0] + (-cry*crz-srx*sry*srz)*pt_list[i][1])*coeff_list[i][0]
                        + (crx*crz*pt_list[i][0] - crx*srz*pt_list[i][1]) * coeff_list[i][1]
                        + ((sry*srz + cry*crz*srx)*pt_list[i][0] + (crz*sry-cry*srx*srz)*pt_list[i][1])*coeff_list[i][2]
                    
                    A_tmp[0, 3] = coeff_list[i][0]
                    A_tmp[0, 4] = coeff_list[i][1]
                    A_tmp[0, 5] = coeff_list[i][2]

                    B_tmp[0,0] = -coeff_list[i][3]

                    A_mat.append(A_tmp)
                    B_mat.append(B_tmp)

                A_mat = np.vstack(A_mat)
                B_mat = np.vstack(B_mat)

                delta_transform = np.linalg.solve(np.matmul(A_mat.T, A_mat), np.matmul(A_mat.T, B_mat))

                if iter_num == 0:
                    # TODO: Degenerate setting
                
                self.transform += np.squeeze(delta_transform)

                delta_r = np.linalg.norm(np.rad2deg(X_mat[:3]))
                delta_t = np.linalg.norm(X_mat[3:] * 100)

                if delta_r < 0.05 and delta_t < 0.05:
                    print("Delta too small.")
                    break

            self.transform_update()
                        
        for i in range(corner_last_ds.shape[0]):
            point_sel = self.point_associate_to_map(corner_last_ds[i, :3])

            cube_i = int((point_sel[0] + 25.0) / 50.0) + self.cloud_center_width
            cube_j = int((point_sel[1] + 25.0) / 50.0) + self.cloud_center_height
            cube_k = int((point_sel[2] + 25.0) / 50.0) + self.cloud_center_depth

            if point_sel[0] + 25 < 0:
                cube_i -= 1
            if point_sel[1] + 25 < 0:
                cube_j -= 1
            if point_sel[2] + 25 < 0:
                cube_k -= 1
            
            if cube_i >=0 and cube_i < self.CLOUD_WIDTH and cube_j >= 0 and cube_j < self.CLOUD_HEIGHT and cube_k >= 0 and cube_k < self.CLOUD_DEPTH:
                cube_ind = cube_i + cube_j * self.CLOUD_WIDTH + cube_k * self.CLOUD_WIDTH * self.CLOUD_HEIGHT
                # TODO: Push to corner array

        for i in range(surf_last_ds.shape[0]):
             point_sel = self.point_associate_to_map(corner_last_ds[i, :3])

            cube_i = int((point_sel[0] + 25.0) / 50.0) + self.cloud_center_width
            cube_j = int((point_sel[1] + 25.0) / 50.0) + self.cloud_center_height
            cube_k = int((point_sel[2] + 25.0) / 50.0) + self.cloud_center_depth

            if point_sel[0] + 25 < 0:
                cube_i -= 1
            if point_sel[1] + 25 < 0:
                cube_j -= 1
            if point_sel[2] + 25 < 0:
                cube_k -= 1
            
            if cube_i >=0 and cube_i < self.CLOUD_WIDTH and cube_j >= 0 and cube_j < self.CLOUD_HEIGHT and cube_k >= 0 and cube_k < self.CLOUD_DEPTH:
                cube_ind = cube_i + cube_j * self.CLOUD_WIDTH + cube_k * self.CLOUD_WIDTH * self.CLOUD_HEIGHT
                # TODO: Push to surface array







        


        