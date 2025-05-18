#!/usr/bin/env python

import rosbag
import sys
import os
from datetime import datetime
import numpy as np

def calculate_H_data(bag_path):
    """
    計算 H_data，即三個 tb 的 utility.data 總和
    
    Args:
        bag_path (str): bag 文件路徑
        
    Returns:
        tuple: (timestamps, H_data) 時間戳和對應的 H_data 值列表
    """
    # 創建空列表來存儲數據
    timestamps = []
    h1_data = []
    h2_data = []
    h3_data = []
    
    # 打開 rosbag 文件
    with rosbag.Bag(bag_path, 'r') as bag:
        try:
            # 獲取三個不同話題的訊息
            h1_msgs = {}
            h2_msgs = {}
            h3_msgs = {}
            
            # 先讀取所有訊息到字典中，以時間戳為鍵
            for topic, msg, t in bag.read_messages(topics=['/tb_1/utility', '/tb_2/utility', '/tb_3/utility']):
                t_sec = t.to_sec()
                if topic == '/tb_1/utility':
                    h1_msgs[t_sec] = msg.data
                elif topic == '/tb_2/utility':
                    h2_msgs[t_sec] = msg.data
                elif topic == '/tb_3/utility':
                    h3_msgs[t_sec] = msg.data
            
            # 取得所有時間戳
            all_timestamps = sorted(set(list(h1_msgs.keys()) + list(h2_msgs.keys()) + list(h3_msgs.keys())))
            
            # 對於每個時間戳，計算 H_data
            for t in all_timestamps:
                h1 = h1_msgs.get(t, 0)
                h2 = h2_msgs.get(t, 0)
                h3 = h3_msgs.get(t, 0)
                
                # 將時間戳和各個數據添加到列表中
                timestamps.append(t)
                h1_data.append(h1)
                h2_data.append(h2)
                h3_data.append(h3)
            
        except Exception as e:
            print(f"讀取訊息時發生錯誤: {str(e)}")
            print("查看 bag 檔案的話題資訊...")
            
            topics_info = bag.get_type_and_topic_info()
            utility_topics = [t for t in topics_info.topics.keys() if 'utility' in t.lower()]
            
            print(f"找到以下 utility 相關話題: {utility_topics}")
            
            if utility_topics:
                print("嘗試從這些話題讀取數據...")
                for topic, msg, t in bag.read_messages(topics=utility_topics):
                    try:
                        print(f"讀取話題: {topic}")
                        print(f"訊息類型: {type(msg)}")
                        print(f"訊息內容: {msg}")
                        break  # 只打印第一條訊息作為範例
                    except Exception as inner_e:
                        print(f"處理訊息時發生錯誤: {str(inner_e)}")
    
    # 計算 H_data (三個 tb 的 utility.data 總和)
    H_data = [h1 + h2 + h3 for h1, h2, h3 in zip(h1_data, h2_data, h3_data)]
    
    if not timestamps:
        print(f"警告: 在 {bag_path} 中找不到有效的 H_data")
    else:
        print(f"在 {bag_path} 中找到 {len(timestamps)} 個時間點的 H_data")
        print(f"H_data 範圍: {min(H_data) if H_data else 0} 到 {max(H_data) if H_data else 0}")
    
    return timestamps, H_data

def find_H_data_threshold(bag_path, threshold=2.0):
    """
    尋找 H_data 第一次達到或超過閾值的時間戳
    
    Args:
        bag_path (str): bag 文件路徑
        threshold (float): 閾值
        
    Returns:
        float: 第一次達到閾值的時間戳（秒），如果找不到則返回 None
    """
    timestamps, H_data = calculate_H_data(bag_path)
    
    if not timestamps or not H_data:
        return None
    
    # 尋找第一次達到閾值的時間點
    for i, h in enumerate(H_data):
        if h >= threshold:
            print(f"在 {bag_path} 中找到 H_data 第一次達到 {threshold} 的時間點: {datetime.fromtimestamp(timestamps[i])}")
            print(f"對應的 H_data 值: {h}")
            return timestamps[i]
    
    print(f"警告: 在 {bag_path} 中找不到 H_data 達到 {threshold} 的時間點")
    return None

def get_bag_duration(bag_path):
    """
    獲取 bag 文件的時間戳範圍
    
    Args:
        bag_path (str): bag 文件路徑
        
    Returns:
        tuple: (開始時間, 結束時間, 持續時間) 單位為秒
    """
    with rosbag.Bag(bag_path, 'r') as bag:
        # 初始化開始和結束時間
        start_time = None
        end_time = None
        
        # 遍歷所有消息以獲取時間範圍
        for _, _, t in bag.read_messages():
            t_sec = t.to_sec()
            
            if start_time is None or t_sec < start_time:
                start_time = t_sec
                
            if end_time is None or t_sec > end_time:
                end_time = t_sec
                
    duration = end_time - start_time
    return start_time, end_time, duration

def trim_bag(input_bag_path, output_bag_path, start_time, end_time):
    """
    修剪 bag 文件到指定的時間範圍
    
    Args:
        input_bag_path (str): 輸入的 bag 文件路徑
        output_bag_path (str): 輸出的 bag 文件路徑
        start_time (float): 開始時間戳（秒）
        end_time (float): 結束時間戳（秒）
    """
    with rosbag.Bag(input_bag_path, 'r') as inbag:
        with rosbag.Bag(output_bag_path, 'w') as outbag:
            # 只寫入指定時間範圍內的消息
            for topic, msg, t in inbag.read_messages():
                t_sec = t.to_sec()
                if t_sec >= start_time and t_sec <= end_time:
                    outbag.write(topic, msg, t)

def format_time(seconds):
    """將秒數格式化為可讀的時間格式"""
    m, s = divmod(seconds, 60)
    h, m = divmod(m, 60)
    return f"{int(h):02d}:{int(m):02d}:{s:.2f}"

def main():
    if len(sys.argv) != 4:
        print("使用方法: python trim_two_bags.py 第一個檔案.bag 第二個檔案.bag 輸出目錄")
        sys.exit(1)
    
    bag1_path = sys.argv[1]
    bag2_path = sys.argv[2]
    output_dir = sys.argv[3]
    
    # 閾值設定為 2.0，按照要求
    threshold = 0.5
    
    # 確保輸出目錄存在
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # 獲取輸出文件名（保留原始名稱，但添加 _trimmed 後綴）
    bag1_name = os.path.basename(bag1_path).replace('.bag', '_trimmed.bag')
    bag2_name = os.path.basename(bag2_path).replace('.bag', '_trimmed.bag')
    
    output_bag1 = os.path.join(output_dir, bag1_name)
    output_bag2 = os.path.join(output_dir, bag2_name)
    
    try:
        # 獲取兩個 bag 的時間信息
        print("獲取 bag 文件時間範圍...")
        bag1_start, bag1_end, bag1_duration = get_bag_duration(bag1_path)
        bag2_start, bag2_end, bag2_duration = get_bag_duration(bag2_path)
        
        # 顯示兩個 bag 的原始時間信息
        print(f"Bag 1 原始時間範圍: {datetime.fromtimestamp(bag1_start)} 到 {datetime.fromtimestamp(bag1_end)}")
        print(f"Bag 2 原始時間範圍: {datetime.fromtimestamp(bag2_start)} 到 {datetime.fromtimestamp(bag2_end)}")
        
        # 尋找兩個 bag 中 H_data 達到閾值的時間點
        print(f"正在尋找 H_data (三個 tb utility.data 的總和) 達到 {threshold} 的時間點...")
        
        threshold_time1 = find_H_data_threshold(bag1_path, threshold=threshold)
        threshold_time2 = find_H_data_threshold(bag2_path, threshold=threshold)
        
        # 設置新的開始時間
        if threshold_time1 is None:
            print(f"在 Bag 1 中找不到 H_data 達到 {threshold} 的時間點，將使用原始開始時間")
            new_start_time1 = bag1_start
        else:
            # 設置 Bag 1 的開始時間為 H_data 達到閾值的前 0.5 秒
            new_start_time1 = threshold_time1 - 0.5
            print(f"設置 Bag 1 開始時間為 H_data 達到 {threshold} 的前 0.5 秒: {datetime.fromtimestamp(new_start_time1)}")
            
        if threshold_time2 is None:
            print(f"在 Bag 2 中找不到 H_data 達到 {threshold} 的時間點，將使用原始開始時間")
            new_start_time2 = bag2_start
        else:
            # 設置 Bag 2 的開始時間為 H_data 達到閾值的前 0.5 秒
            new_start_time2 = threshold_time2 - 0.5
            print(f"設置 Bag 2 開始時間為 H_data 達到 {threshold} 的前 0.5 秒: {datetime.fromtimestamp(new_start_time2)}")
        
        # 確保新的開始時間不早於原始開始時間
        if new_start_time1 < bag1_start:
            print(f"警告: 計算的開始時間早於 Bag 1 的原始開始時間，將使用原始開始時間")
            new_start_time1 = bag1_start
        if new_start_time2 < bag2_start:
            print(f"警告: 計算的開始時間早於 Bag 2 的原始開始時間，將使用原始開始時間")
            new_start_time2 = bag2_start
        
        # 計算新的持續時間
        new_duration1 = bag1_end - new_start_time1
        new_duration2 = bag2_end - new_start_time2
        
        # 顯示調整後的時間信息
        print(f"Bag 1: {bag1_path}")
        print(f"  原始開始時間: {datetime.fromtimestamp(bag1_start)}")
        print(f"  新開始時間: {datetime.fromtimestamp(new_start_time1)}")
        print(f"  結束時間: {datetime.fromtimestamp(bag1_end)}")
        print(f"  新持續時間: {format_time(new_duration1)}")
        print()
        
        print(f"Bag 2: {bag2_path}")
        print(f"  原始開始時間: {datetime.fromtimestamp(bag2_start)}")
        print(f"  新開始時間: {datetime.fromtimestamp(new_start_time2)}")
        print(f"  結束時間: {datetime.fromtimestamp(bag2_end)}")
        print(f"  新持續時間: {format_time(new_duration2)}")
        print()
        
        # 找出較短的持續時間
        if new_duration1 <= new_duration2:
            shorter_duration = new_duration1
            print(f"新的 Bag 1 較短，將使用其持續時間: {format_time(shorter_duration)}")
        else:
            shorter_duration = new_duration2
            print(f"新的 Bag 2 較短，將使用其持續時間: {format_time(shorter_duration)}")
        
        # 計算新的結束時間（從各自的新開始時間起）
        bag1_new_end = new_start_time1 + shorter_duration
        bag2_new_end = new_start_time2 + shorter_duration
        
        # 修剪兩個 bag
        print(f"正在修剪 Bag 1...")
        trim_bag(bag1_path, output_bag1, new_start_time1, bag1_new_end)
        
        print(f"正在修剪 Bag 2...")
        trim_bag(bag2_path, output_bag2, new_start_time2, bag2_new_end)
        
        print(f"修剪完成！")
        print(f"輸出文件:")
        print(f"  Bag 1: {output_bag1}")
        print(f"  Bag 2: {output_bag2}")
        
    except Exception as e:
        print(f"處理過程中發生錯誤: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()