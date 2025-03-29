#!/usr/bin/env python

import rosbag
import sys
import os
from datetime import datetime

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

def trim_bag(input_bag_path, output_bag_path, end_time):
    """
    修剪 bag 文件到指定的結束時間
    
    Args:
        input_bag_path (str): 輸入的 bag 文件路徑
        output_bag_path (str): 輸出的 bag 文件路徑
        end_time (float): 結束時間戳（秒）
    """
    with rosbag.Bag(input_bag_path, 'r') as inbag:
        with rosbag.Bag(output_bag_path, 'w') as outbag:
            # 只寫入指定時間之前的消息
            for topic, msg, t in inbag.read_messages():
                if t.to_sec() <= end_time:
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
        bag1_start, bag1_end, bag1_duration = get_bag_duration(bag1_path)
        bag2_start, bag2_end, bag2_duration = get_bag_duration(bag2_path)
        
        # 顯示兩個 bag 的時間信息
        print(f"Bag 1: {bag1_path}")
        print(f"  開始時間: {datetime.fromtimestamp(bag1_start)}")
        print(f"  結束時間: {datetime.fromtimestamp(bag1_end)}")
        print(f"  持續時間: {format_time(bag1_duration)}")
        print()
        
        print(f"Bag 2: {bag2_path}")
        print(f"  開始時間: {datetime.fromtimestamp(bag2_start)}")
        print(f"  結束時間: {datetime.fromtimestamp(bag2_end)}")
        print(f"  持續時間: {format_time(bag2_duration)}")
        print()
        
        # 找出較短的持續時間
        if bag1_duration <= bag2_duration:
            shorter_duration = bag1_duration
            shorter_end = bag1_end
            print(f"Bag 1 較短，將使用其持續時間: {format_time(shorter_duration)}")
        else:
            shorter_duration = bag2_duration
            shorter_end = bag2_end
            print(f"Bag 2 較短，將使用其持續時間: {format_time(shorter_duration)}")
        
        # 計算新的結束時間（從各自的開始時間起）
        bag1_new_end = bag1_start + shorter_duration
        bag2_new_end = bag2_start + shorter_duration
        
        # 修剪兩個 bag
        print(f"正在修剪 Bag 1...")
        trim_bag(bag1_path, output_bag1, bag1_new_end)
        
        print(f"正在修剪 Bag 2...")
        trim_bag(bag2_path, output_bag2, bag2_new_end)
        
        print(f"修剪完成！")
        print(f"輸出文件:")
        print(f"  Bag 1: {output_bag1}")
        print(f"  Bag 2: {output_bag2}")
        
    except Exception as e:
        print(f"處理過程中發生錯誤: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()
