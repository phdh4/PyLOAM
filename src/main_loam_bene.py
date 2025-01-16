from data_loader import NPYLoader
from laser_odometry import Odometry
from laser_mapping import Mapper
import numpy as np

if __name__== '__main__':
    loader = NPYLoader(path="../utils/points_npy/", name='NSH indoor')
    config = None
    odometry = Odometry(config=config)
    mapper = Mapper(config=config)
    trans_list = []
    for i in range(len(loader)):
        cloud = loader[i]
        ring = cloud[:, 6]*4+cloud[:, 7]
        ring = np.expand_dims(ring, axis=1)
        time_stamp = cloud[:, 4]+cloud[:, 5]*1e-9
        rel_time = (time_stamp - time_stamp.min()) / (time_stamp.max() - time_stamp.min())
        rel_time = np.expand_dims(rel_time, axis=1)
        cloud_bene = np.hstack((cloud[:, 0:4], ring))
        cloud_bene = np.hstack((cloud_bene, rel_time))

        surf_pts, corner_pts, odom = odometry.grab_frame(cloud_bene)
        trans = mapper.map_frame(odom, corner_pts, surf_pts)
        trans_list.append(trans)
