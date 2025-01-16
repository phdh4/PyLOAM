import av
import os
import glob

import numpy as np

# 视频文件路径
video_path = 'E:/hzy/data/depth_predict/long_20241221_110000_10_8008_06.mp4'

# # 读取并排序pcd标签文件名
pcd_list = glob.glob("./*.pcd")

# 视频起始NTP时间戳
ntp_start = 1734750000  # 视频开始时的 NTP 时间戳（单位：秒）
container = av.open(video_path)
start_time = container.start_time  # 容器的起始时间（单位是微秒）

# 提取标签文件名（秒.纳秒.pcd）对应的 NTP 时间戳
pcd_ntps = [(int(f[2:].split('.')[-3]) + int(f[2:].split('.')[-2])*1e-9) for f in pcd_list]
name = []
# 双指针遍历视频帧和标签文件
label_index = 0
diff = np.Inf
for frame in container.decode(video=0):
    relative_timestamp = frame.time  # 相对时间戳（单位：秒）
    # 计算该帧的绝对时间戳（单位：秒）
    container_absolute_timestamp = start_time / 1e6 + relative_timestamp
    # 计算该帧的 NTP 时间
    frame_ntp = ntp_start + container_absolute_timestamp

    # 处理当前pcd标签文件，pcd标签文件处理完则结束循环
    while label_index < len(pcd_ntps): # and pcd_ntps[label_index] <= frame_ntp:
        # 当前标签的时间戳
        label_ntp = pcd_ntps[label_index]
        # 计算与当前标签的 NTP 时间戳的差值，如果插值变小，说明可以保存图像，同时覆盖之前保存的图像（如果有）
        if diff > abs(frame_ntp - label_ntp):
            diff = abs(frame_ntp - label_ntp)
            if diff < 0.1:
                img = frame.to_image()
                img.save(f"{label_ntp}.png")
                print(f"保存了与 NTP 时间戳 {label_ntp} 最近的帧为 '{label_ntp}.png'。，该帧时间戳为{frame_ntp}")
            break
        else:  # 如果差值变大，说明已经找到最近的图像了，那么label_index + 1， diff重置
            label_index += 1
            diff = np.Inf
            break
    if label_index >= len(pcd_ntps):
        break