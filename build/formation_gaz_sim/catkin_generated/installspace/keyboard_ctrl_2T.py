#!/usr/bin/env python3

import rospy
import sys
import termios
import tty
import threading
import time
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest

class KeyboardController:
    def __init__(self):
        rospy.init_node('keyboard_uav_controller')
        
        # 目標無人機的命名空間
        self.target_namespaces = ['target_1', 'target_2']
        
        # 儲存每台無人機的狀態
        self.uav_states = {}
        self.uav_poses = {}
        self.current_targets = {}
        
        # 初始化參數
        self.altitude = 2.0  # 默認懸停高度 (meters)
        self.active_uav = 0  # 當前控制的無人機 (0 或 1)
        self.is_running = True
        self.is_trajectory_active = False
        
        # 為每台無人機創建發布者、訂閱者和服務客戶端
        self.setup_uav_connections()
        
        # 預設軌跡 - 圓形路徑
        self.circle_radius = 3.0  # 圓半徑，單位：米
        self.circle_speed = 1.0   # 速度，單位：米/秒
        
        rospy.loginfo("鍵盤控制器已初始化")
        rospy.loginfo("控制說明:")
        rospy.loginfo("1-2: 選擇控制的無人機 (1: target_1, 2: target_2)")
        rospy.loginfo("a: Arm 無人機")
        rospy.loginfo("d: Disarm 無人機")
        rospy.loginfo("t: 起飛到預設高度")
        rospy.loginfo("l: 降落")
        rospy.loginfo("c: 執行圓形軌跡")
        rospy.loginfo("8: 向前飛行")
        rospy.loginfo("2: 向後飛行")
        rospy.loginfo("4: 向左飛行")
        rospy.loginfo("6: 向右飛行")
        rospy.loginfo("5: 停止移動")
        rospy.loginfo("q: 退出程式")
        
    def setup_uav_connections(self):
        # 為每台無人機設置發布者、訂閱者和服務
        for idx, ns in enumerate(self.target_namespaces):
            # 存儲初始位置
            initial_pose = PoseStamped()
            if idx == 0:  # target_1
                initial_pose.pose.position.x = 12.0
                initial_pose.pose.position.y = 12.0
            else:  # target_2
                initial_pose.pose.position.x = 13.0
                initial_pose.pose.position.y = 13.0
            initial_pose.pose.position.z = 0.0
            self.current_targets[ns] = initial_pose
            
            # 設置發布者 - 發送位置指令
            setpoint_pub = rospy.Publisher(
                f'/{ns}/mavros/setpoint_position/local', 
                PoseStamped, 
                queue_size=10
            )
            
            # 設置速度發布者
            velocity_pub = rospy.Publisher(
                f'/{ns}/mavros/setpoint_velocity/cmd_vel',
                TwistStamped,
                queue_size=10
            )
            
            # 訂閱無人機當前狀態
            rospy.Subscriber(
                f'/{ns}/mavros/state', 
                State, 
                self.state_callback, 
                callback_args=ns
            )
            
            # 訂閱無人機當前位置
            rospy.Subscriber(
                f'/{ns}/mavros/local_position/pose', 
                PoseStamped, 
                self.pose_callback, 
                callback_args=ns
            )
            
            # 設置服務客戶端 - 用於解鎖(arm)
            arm_client = rospy.ServiceProxy(
                f'/{ns}/mavros/cmd/arming', 
                CommandBool
            )
            
            # 設置服務客戶端 - 用於更改飛行模式
            mode_client = rospy.ServiceProxy(
                f'/{ns}/mavros/set_mode', 
                SetMode
            )
            
            # 設置服務客戶端 - 用於起飛
            takeoff_client = rospy.ServiceProxy(
                f'/{ns}/mavros/cmd/takeoff', 
                CommandTOL
            )
            
            # 設置服務客戶端 - 用於降落
            land_client = rospy.ServiceProxy(
                f'/{ns}/mavros/cmd/land', 
                CommandTOL
            )
            
            # 存儲所有的發布者和服務客戶端
            self.uav_states[ns] = {
                'state': None,
                'setpoint_pub': setpoint_pub,
                'velocity_pub': velocity_pub,
                'arm_client': arm_client,
                'mode_client': mode_client,
                'takeoff_client': takeoff_client,
                'land_client': land_client
            }
            
            rospy.loginfo(f"已設置 {ns} 的所有連接")
    
    def state_callback(self, msg, ns):
        """儲存無人機的狀態"""
        self.uav_states[ns]['state'] = msg
    
    def pose_callback(self, msg, ns):
        """儲存無人機的位置"""
        self.uav_poses[ns] = msg
    
    def arm_uav(self, ns):
        """解鎖指定的無人機"""
        if self.uav_states[ns]['state'] is None:
            rospy.logwarn(f"{ns} 狀態不可用")
            return False
        
        # 檢查是否已經處於解鎖狀態
        if self.uav_states[ns]['state'].armed:
            rospy.loginfo(f"{ns} 已經解鎖")
            return True
        
        # 首先設置為OFFBOARD模式 (PX4中相當於ArduPilot的GUIDED模式)
        mode_req = SetModeRequest()
        mode_req.custom_mode = 'OFFBOARD'  # 將GUIDED改為OFFBOARD
        mode_response = self.uav_states[ns]['mode_client'](mode_req)
        
        if not mode_response.mode_sent:
            rospy.logwarn(f"無法將 {ns} 設置為OFFBOARD模式")
            return False
        
        rospy.loginfo(f"{ns} 已設置為OFFBOARD模式")
        
        # 解鎖無人機
        arm_req = CommandBoolRequest()
        arm_req.value = True
        
        # 重試幾次，因為有時解鎖可能失敗
        for i in range(3):
            arm_response = self.uav_states[ns]['arm_client'](arm_req)
            if arm_response.success:
                rospy.loginfo(f"{ns} 解鎖成功")
                return True
            rospy.sleep(1)
        
        rospy.logwarn(f"無法解鎖 {ns}")
        return False
    
    def disarm_uav(self, ns):
        """上鎖指定的無人機"""
        if self.uav_states[ns]['state'] is None:
            rospy.logwarn(f"{ns} 狀態不可用")
            return False
        
        # 檢查是否已經處於上鎖狀態
        if not self.uav_states[ns]['state'].armed:
            rospy.loginfo(f"{ns} 已經上鎖")
            return True
        
        # 上鎖無人機
        arm_req = CommandBoolRequest()
        arm_req.value = False
        
        arm_response = self.uav_states[ns]['arm_client'](arm_req)
        if arm_response.success:
            rospy.loginfo(f"{ns} 上鎖成功")
            return True
        
        rospy.logwarn(f"無法上鎖 {ns}")
        return False
    
    def takeoff(self, ns):
        """讓指定的無人機起飛到指定高度"""
        if self.uav_states[ns]['state'] is None:
            rospy.logwarn(f"{ns} 狀態不可用")
            return False
        
        # 檢查無人機是否已解鎖
        if not self.uav_states[ns]['state'].armed:
            rospy.logwarn(f"{ns} 尚未解鎖，無法起飛")
            return False
        
        # 使用MAVROS的起飛服務
        takeoff_req = CommandTOLRequest()
        takeoff_req.altitude = self.altitude
        takeoff_req.latitude = 0  # 當前位置
        takeoff_req.longitude = 0  # 當前位置
        
        takeoff_response = self.uav_states[ns]['takeoff_client'](takeoff_req)
        if takeoff_response.success:
            rospy.loginfo(f"{ns} 正在起飛到 {self.altitude} 米高度")
            return True
        
        rospy.logwarn(f"{ns} 起飛失敗")
        return False
    
    def land(self, ns):
        """讓指定的無人機降落"""
        if self.uav_states[ns]['state'] is None:
            rospy.logwarn(f"{ns} 狀態不可用")
            return False
        
        # 使用MAVROS的降落服務
        land_req = CommandTOLRequest()
        
        land_response = self.uav_states[ns]['land_client'](land_req)
        if land_response.success:
            rospy.loginfo(f"{ns} 正在降落")
            return True
        
        rospy.logwarn(f"{ns} 降落失敗")
        return False
    
    def set_velocity(self, ns, vx, vy, vz):
        """設置無人機的速度"""
        if ns not in self.uav_states:
            rospy.logwarn(f"無人機 {ns} 不存在")
            return
        
        # 創建速度訊息
        vel_msg = TwistStamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        vel_msg.twist.linear.z = vz
        
        # 發布速度指令
        self.uav_states[ns]['velocity_pub'].publish(vel_msg)
    
    def execute_circle_trajectory(self, ns):
        """執行圓形軌跡"""
        if ns not in self.uav_poses:
            rospy.logwarn(f"無人機 {ns} 的位置不可用")
            return
        
        # 獲取當前位置作為圓心
        center_x = self.uav_poses[ns].pose.position.x
        center_y = self.uav_poses[ns].pose.position.y
        current_z = self.uav_poses[ns].pose.position.z
        
        rospy.loginfo(f"開始執行 {ns} 的圓形軌跡，圓心: ({center_x}, {center_y}), 高度: {current_z}")
        
        # 標記軌跡開始
        self.is_trajectory_active = True
        
        # 計算圓形一周的時間
        circle_circumference = 2 * math.pi * self.circle_radius
        circle_time = circle_circumference / self.circle_speed
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)  # 20Hz
        
        try:
            while self.is_trajectory_active and not rospy.is_shutdown():
                # 計算當前時間在圓上的位置
                elapsed = (rospy.Time.now() - start_time).to_sec()
                angle = (elapsed % circle_time) / circle_time * 2 * math.pi
                
                # 計算目標位置
                target_x = center_x + self.circle_radius * math.cos(angle)
                target_y = center_y + self.circle_radius * math.sin(angle)
                
                # 創建位置訊息
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.pose.position.x = target_x
                pose_msg.pose.position.y = target_y
                pose_msg.pose.position.z = current_z  # 保持高度不變
                
                # 發布位置指令
                self.uav_states[ns]['setpoint_pub'].publish(pose_msg)
                
                rate.sleep()
        except Exception as e:
            rospy.logerr(f"執行圓形軌跡時發生錯誤: {e}")
        finally:
            self.is_trajectory_active = False
            rospy.loginfo(f"{ns} 的圓形軌跡執行完成")
    
    def keyboard_control(self):
        """處理鍵盤輸入"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while self.is_running and not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    
                    # 處理按鍵命令
                    if key == 'q':
                        self.is_running = False
                        self.is_trajectory_active = False
                        rospy.loginfo("退出程式...")
                    
                    elif key in ['1', '2']:
                        self.active_uav = int(key) - 1
                        active_ns = self.target_namespaces[self.active_uav]
                        rospy.loginfo(f"切換到控制 {active_ns}")
                    
                    else:
                        # 取得當前選擇的無人機
                        ns = self.target_namespaces[self.active_uav]
                        
                        if key == 'a':
                            self.arm_uav(ns)
                        
                        elif key == 'd':
                            self.disarm_uav(ns)
                        
                        elif key == 't':
                            self.takeoff(ns)
                        
                        elif key == 'l':
                            self.land(ns)
                        
                        elif key == 'c':
                            if not self.is_trajectory_active:
                                # 啟動新線程執行軌跡，避免阻塞鍵盤控制
                                trajectory_thread = threading.Thread(
                                    target=self.execute_circle_trajectory,
                                    args=(ns,)
                                )
                                trajectory_thread.daemon = True
                                trajectory_thread.start()
                            else:
                                self.is_trajectory_active = False
                                rospy.loginfo("停止軌跡執行")
                        
                        # 基本的方向控制 (WASD)
                        elif key == '8':  # 前進
                            self.set_velocity(ns, 1.0, 0.0, 0.0)
                        
                        elif key == '2':  # 後退
                            self.set_velocity(ns, -1.0, 0.0, 0.0)
                        
                        elif key == '4':  # 左移
                            self.set_velocity(ns, 0.0, 1.0, 0.0)
                        
                        elif key == '6':  # 右移
                            self.set_velocity(ns, 0.0, -1.0, 0.0)
                        
                        elif key == '5':  # 停止
                            self.set_velocity(ns, 0.0, 0.0, 0.0)
                
                rospy.sleep(0.1)  # 稍微休息，避免CPU占用過高
        
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            
            # 在退出前，嘗試將所有無人機降落
            for ns in self.target_namespaces:
                self.land(ns)

if __name__ == '__main__':
    try:
        # 引入select模塊，用於非阻塞的鍵盤輸入
        import select
        
        controller = KeyboardController()
        controller.keyboard_control()
    
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"發生錯誤: {e}")
        import traceback
        traceback.print_exc()