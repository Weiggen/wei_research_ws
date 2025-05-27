#!/usr/bin/env python3

import rospy
import math
import threading
import sys
import tty
import termios
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class UAVController:
    def __init__(self):
        rospy.init_node('control_target_1', anonymous=True)
        
        # 參數設定
        self.drone_name = 'target_1'
        self.state = State()
        self.current_pose = PoseStamped()
        self.rate = rospy.Rate(20)  # 20Hz
        
        # 圓形軌跡參數
        self.circle_radius = 5.0  # 5m 半徑
        self.circle_z = 0.8       # 3m 高度
        self.circle_center_x = 0.0
        self.circle_center_y = 0.0
        self.angular_speed = 0.07  # rad/s
        
        # ROS訂閱與發布
        state_sub = rospy.Subscriber(f"/{self.drone_name}/mavros/state", State, self.state_callback)
        pose_sub = rospy.Subscriber(f"/{self.drone_name}/mavros/local_position/pose", PoseStamped, self.pose_callback)
        
        self.local_pos_pub = rospy.Publisher(f"/{self.drone_name}/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        # ROS服務
        rospy.wait_for_service(f"/{self.drone_name}/mavros/cmd/arming")
        rospy.wait_for_service(f"/{self.drone_name}/mavros/set_mode")
        self.arming_client = rospy.ServiceProxy(f"/{self.drone_name}/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy(f"/{self.drone_name}/mavros/set_mode", SetMode)
        
        # 等待連接
        rospy.loginfo("等待飛控連接...")
        while not rospy.is_shutdown() and not self.state.connected:
            self.rate.sleep()
        rospy.loginfo("飛控已連接!")
        
        # 發送初始位置點(預熱)
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0
        self.target_pose.pose.position.y = 0
        self.target_pose.pose.position.z = 0
        
        # 預熱階段，發送一些位置點
        rospy.loginfo("進行OFFBOARD模式預熱...")
        for i in range(100):
            self.local_pos_pub.publish(self.target_pose)
            self.rate.sleep()

    # 狀態回調函數
    def state_callback(self, msg):
        self.state = msg
        
    # 位置回調函數
    def pose_callback(self, msg):
        self.current_pose = msg
        
    # 起飛函數
    def takeoff(self):
        rospy.loginfo("請求解鎖...")
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        
        last_request = rospy.Time.now()
        
        # 嘗試解鎖並切換模式
        while not rospy.is_shutdown():
            if self.state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                rospy.loginfo("請求OFFBOARD模式...")
                offb_set_mode = SetModeRequest()
                offb_set_mode.custom_mode = "OFFBOARD"
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD模式已發送")
                last_request = rospy.Time.now()
            elif not self.state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                if self.arming_client.call(arm_cmd).success:
                    rospy.loginfo("無人機已解鎖")
                last_request = rospy.Time.now()
            
            # 發送起飛位置
            self.target_pose.header.stamp = rospy.Time.now()
            self.target_pose.pose.position.z = self.circle_z  # 起飛高度
            self.local_pos_pub.publish(self.target_pose)
            
            # 如果已經解鎖並在OFFBOARD模式且接近目標高度，則完成起飛
            if self.state.armed and self.state.mode == "OFFBOARD" and abs(self.current_pose.pose.position.z - self.circle_z) < 0.1:
                rospy.loginfo("起飛完成，到達目標高度")
                return True
                
            self.rate.sleep()
            
    # 執行圓形軌跡
    def execute_circular_trajectory(self):
        rospy.loginfo("開始執行圓形軌跡...")
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # 計算當前時間
            elapsed = (rospy.Time.now() - start_time).to_sec()
            
            # 計算圓形軌跡上的位置
            angle = self.angular_speed * elapsed
            self.target_pose.header.stamp = rospy.Time.now()
            self.target_pose.pose.position.x = self.circle_center_x + self.circle_radius * math.cos(angle)
            self.target_pose.pose.position.y = self.circle_center_y + self.circle_radius * math.sin(angle)
            self.target_pose.pose.position.z = self.circle_z
            
            # 發送位置指令
            self.local_pos_pub.publish(self.target_pose)
            self.rate.sleep()
            
    # 懸停在定高
    def hover_at_altitude(self):
        rospy.loginfo("已到達目標高度，現在懸停定高...")
        while not rospy.is_shutdown() and not self.start_trajectory:
            # 保持在當前位置懸停
            self.target_pose.header.stamp = rospy.Time.now()
            self.target_pose.pose.position.x = 0
            self.target_pose.pose.position.y = 0
            self.target_pose.pose.position.z = self.circle_z
            self.local_pos_pub.publish(self.target_pose)
            self.rate.sleep()
        
        # 如果收到開始信號，開始執行圓形軌跡
        if self.start_trajectory:
            rospy.loginfo("收到開始信號，準備執行圓形軌跡...")
            return True
        return False
    
    # 監聽鍵盤輸入
    def keyboard_listener(self):
        def get_key():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch
        
        rospy.loginfo("等待按下 's' 鍵開始執行圓形軌跡...")
        while not rospy.is_shutdown():
            key = get_key()
            if key == 's':
                rospy.loginfo("收到 's' 鍵，開始執行軌跡")
                self.start_trajectory = True
                break
            elif key == '\x03':  # Ctrl+C
                rospy.signal_shutdown("用戶中斷")
                break
                
    # 主程序運行
    def run(self):
        try:
            rospy.loginfo("開始執行target_1控制節點...")
            # 初始化開始軌跡標記為False
            self.start_trajectory = False
            
            # 啟動鍵盤監聽線程
            keyboard_thread = threading.Thread(target=self.keyboard_listener)
            keyboard_thread.daemon = True
            keyboard_thread.start()
            
            # 執行起飛
            success = self.takeoff()
            if success:
                # 定高懸停，等待鍵盤指令
                if self.hover_at_altitude():
                    # 收到開始信號後執行圓形軌跡
                    self.execute_circular_trajectory()
                    
        except rospy.ROSInterruptException:
            rospy.loginfo("程序被中斷")

if __name__ == '__main__':
    try:
        controller = UAVController()
        controller.run()
    except rospy.ROSInterruptException:
        pass