#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

def process_rosbag(bag_file):
    # 創建空列表來存儲數據
    timestamps = []
    h1_data = []
    h2_data = []
    h3_data = []
    H_data = []

    # 打開rosbag文件
    with rosbag.Bag(bag_file, 'r') as bag:
        # 讀取三個不同話題的數據
        h1_msgs = bag.read_messages(topics=['/iris_1/utility'])
        h2_msgs = bag.read_messages(topics=['/iris_2/utility'])
        h3_msgs = bag.read_messages(topics=['/iris_3/utility'])

        # 處理每個消息
        for h1_msg, h2_msg, h3_msg in zip(h1_msgs, h2_msgs, h3_msgs):
            # 獲取時間戳
            t = h1_msg.timestamp.to_sec()
            timestamps.append(t)

            # 獲取數據
            h1 = h1_msg.message.data
            h2 = h2_msg.message.data
            h3 = h3_msg.message.data

            # 存儲單獨的數據
            h1_data.append(h1)
            h2_data.append(h2)
            h3_data.append(h3)

            # 計算總和 H
            H = h1 + h2 + h3
            H_data.append(H)

    return timestamps, h1_data, h2_data, h3_data, H_data

def plot_data(timestamps, h1_data, h2_data, h3_data, H_data):
    # 創建圖表
    plt.figure(figsize=(12, 8))

    # 繪製所有數據
    plt.plot(timestamps, h1_data, label='h1', linewidth=1, linestyle='--')
    plt.plot(timestamps, h2_data, label='h2', linewidth=1, linestyle='--')
    plt.plot(timestamps, h3_data, label='h3', linewidth=1, linestyle='--')
    plt.plot(timestamps, H_data, label='H (Total)', linewidth=2)

    # 設置圖表屬性
    plt.xlabel('Time (seconds)')
    plt.ylabel('Value')
    plt.title('ROS Bag Data Analysis')
    plt.legend()
    plt.grid(True)

    # 顯示圖表
    plt.show()

def main():
    # 替換為您的rosbag文件路徑
    bag_file = '/home/weiggen/wei_research_ws/src/voronoi_cbsa/bag/1221_D.bag'

    try:
        # 處理數據
        timestamps, h1_data, h2_data, h3_data, H_data = process_rosbag(bag_file)
        
        # 繪製圖表
        plot_data(timestamps, h1_data, h2_data, h3_data, H_data)

    except Exception as e:
        print(f"Error processing bag file: {e}")

if __name__ == "__main__":
    main()
