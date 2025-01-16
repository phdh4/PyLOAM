import glob
import json
import av
import numpy as np


def frame_iter(video):
    #streams = [s for s in video.streams if s.type == b'video']
    #print(streams)
    streams = video.streams.video
    for packet in video.demux(streams):
        # print(packet, packet.buffer_size, packet.buffer_ptr,dir(packet))
        for frame in packet.decode():
            yield frame

def save_dict_to_txt(dictionary, filename):
    with open(filename, 'w') as f:
        for key, value in dictionary.items():
            # 去掉引号并写入键值对，每个键值对后加空行
            f.write(f"{key}\t{value}\n")


if __name__ == "__main__":
    # 视频文件路径
    video_path = 'E:/hzy/data/depth_predict/08/long_20241219_210000_10_8008_06.mp4'
    # 读取并排序pcd标签文件名
    pcd_list = glob.glob("E:/hzy/data/depth_predict/08/pcd/*.pcd")
    # 提取标签文件名（秒.纳秒.pcd）对应的 NTP 时间戳
    # pcd_ntps = [(int(f.split("\\")[-1].split('.')[-3]) + int(f.split("\\")[-1].split('.')[-2]) * 1e-9) for f in pcd_list]
    pcd_names = [f.split("\\")[-1][0:-4] for f in pcd_list]
    name = {}
    # 双指针遍历视频帧和标签文件
    pcd_index = 0
    diff = np.Inf
    video = av.open(video_path)
    frames = frame_iter(video)
    for frame in frames:
        timestamp = frame.side_data.get(av.sidedata.sidedata.Type.SEI_UNREGISTERED)
        if timestamp is not None:
            buf = bytes(timestamp)
            timestamp = buf[-8:]
            timestamp = int.from_bytes(timestamp, byteorder='little') / 1000
            # 处理当前pcd标签文件，pcd标签文件处理完则结束循环
            while pcd_index < len(pcd_names):  # and pcd_ntps[label_index] <= frame_ntp:
                # 当前标签的时间戳
                pcd_name = pcd_names[pcd_index]
                pcd_ntp = int(pcd_name.split('.')[0])+int(pcd_name.split('.')[1]) * 1e-9
                # 计算与当前标签的 NTP 时间戳的差值，如果插值变小，说明可以保存图像，同时覆盖之前保存的图像（如果有）
                if diff > abs(timestamp - pcd_ntp):
                    diff = abs(timestamp - pcd_ntp)
                    if diff < 0.5:
                        img = frame.to_image()
                        img.save("E:/hzy/data/depth_predict/08/image/"+pcd_name+".png")
                        print("保存了与 NTP 时间戳"+ pcd_name +"最近的帧，"+"该帧时间戳为"+str(timestamp))
                        name[pcd_name] = str(timestamp)
                    break
                else:  # 如果差值变大，说明已经找到最近的图像了，那么pcd_index + 1， diff重置
                    pcd_index += 1
                    diff = np.Inf
                    break
            if pcd_index >= len(pcd_names):
                break
    save_dict_to_txt(name, 'data.txt')
