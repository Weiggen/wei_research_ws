#!/usr/bin/env python

import rosbag
import sys
from datetime import datetime

def get_precise_time_info(bag_path):
    """
    獲取bag檔案的精確時間信息
    """
    with rosbag.Bag(bag_path, 'r') as bag:
        # 獲取第一條消息
        first_topic, first_msg, first_time = next(bag.read_messages())
        
        # 初始時間
        start_time = first_time.to_sec()
        
        print(f"\n===== Bag檔案時間信息 =====")
        print(f"起始時間戳: {start_time}")
        
        # 計算80秒位置的時間戳
        target_time = start_time + 60.0
        print(f"60秒位置的時間戳: {target_time}")
        print("\n要使用這個時間戳作為trim_bag.py的結束時間參數")
        print(f"\n使用方法範例:")
        print(f"python trim_bag.py dynamic_sim.bag output.bag {target_time}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("使用方法: python get_precise_time_info.py dynamic_sim.bag")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    get_precise_time_info(bag_path)