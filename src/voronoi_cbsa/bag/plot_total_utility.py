#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

def process_rosbag(bag_file):
    # 創建空列表來存儲數據
    timestamps = []
    h1 = []
    h2 = []
    h3 = []
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

            # 計算總和 H
            H = h1 + h2 + h3
            H_data.append(H)

    # 將時間戳轉換為從0開始的相對時間
    timestamps = np.array(timestamps)
    timestamps = timestamps - timestamps[0]
    
    return timestamps, H_data

def plot_comparison(bag_file1, bag_file2):
    # 創建圖表
    plt.figure(figsize=(12, 8))

    # 處理第一個bag檔案
    timestamps1, H_data1 = process_rosbag(bag_file1)
    
    # 處理第二個bag檔案
    timestamps2, H_data2 = process_rosbag(bag_file2)

    # 繪製兩組H數據
    plt.plot(timestamps1, H_data1, label='H (1221)', linewidth=2)
    plt.plot(timestamps2, H_data2, label='H (Before)', linewidth=2, linestyle='--')

    # 設置圖表屬性
    plt.xlabel('Time (seconds)')
    plt.ylabel('H Value')
    plt.title('Comparison of H Values from Two Bag Files')
    plt.legend()
    plt.grid(True)

    # 顯示圖表
    plt.show()

def main():
    # 替換為您的兩個rosbag文件路徑
    bag_file1 = '/home/weiggen/wei_research_ws/src/voronoi_cbsa/bag/1221_D.bag'
    bag_file2 = '/home/weiggen/wei_research_ws/src/voronoi_cbsa/bag/Dynamic_utilityfrom0_f.bag'

    try:
        # 繪製比較圖表
        plot_comparison(bag_file1, bag_file2)

    except Exception as e:
        print(f"Error processing bag files: {e}")

if __name__ == "__main__":
    main()