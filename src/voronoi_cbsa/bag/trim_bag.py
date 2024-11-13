#!/usr/bin/env python

import rosbag
import sys
from datetime import datetime

def trim_bag(input_bag_path, output_bag_path, end_time=None):
    """
    修剪bag文件，刪除最後一個時間段的數據
    
    Args:
        input_bag_path (str): 輸入的bag文件路徑
        output_bag_path (str): 輸出的bag文件路徑
        end_time (float): 可選，指定結束時間戳（如果不指定，將自動計算）
    """
    
    # 打開輸入的bag文件
    with rosbag.Bag(input_bag_path, 'r') as inbag:
        # 獲取開始和結束時間
        start_time = None
        last_time = None
        
        # 第一次遍歷獲取時間信息
        for topic, msg, t in inbag.read_messages():
            if start_time is None:
                start_time = t.to_sec()
            last_time = t.to_sec()
        
        # 如果沒有指定結束時間，自動計算
        if end_time is None:
            # 獲取總時長
            total_duration = last_time - start_time
            # 設置結束時間為去掉最後一段
            end_time = last_time - (total_duration * 0.1)  # 預設刪除最後10%的數據
        
        # 創建新的bag文件
        with rosbag.Bag(output_bag_path, 'w') as outbag:
            # 第二次遍歷，只寫入指定時間之前的消息
            for topic, msg, t in inbag.read_messages():
                if t.to_sec() <= end_time:
                    outbag.write(topic, msg, t)

def main():
    if len(sys.argv) < 3:
        print("使用方法: python trim_bag.py 輸入檔案.bag 輸出檔案.bag [結束時間]")
        sys.exit(1)
    
    input_bag = sys.argv[1]
    output_bag = sys.argv[2]
    end_time = float(sys.argv[3]) if len(sys.argv) > 3 else None
    
    try:
        trim_bag(input_bag, output_bag, end_time)
        print(f"成功修剪bag文件。輸出保存在: {output_bag}")
    except Exception as e:
        print(f"處理過程中發生錯誤: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()