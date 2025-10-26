#! /usr/bin/env python3
import rospy
import pygame
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from voronoi_cbsa.msg import TargetInfoArray, ValidSensors, WeightArray
from scipy.stats import multivariate_normal
from scipy.spatial.transform import Rotation as R

class Visualization():
    def __init__(self):
        # Initialization robots parameters
        self.total_agents = rospy.get_param("~total_agents", 3)
        self.vehicle = "tb"
        self.agent_pos = {}
        self.agent_per = {}
        self.target_received = False
        self.targets = {}
        
        # Initialize environment variables
        real_map_width = rospy.get_param("~map_width", 24)
        real_map_height = rospy.get_param("~map_height", 24)
        self.map_size = np.array([real_map_width, real_map_height])

        grid_size = rospy.get_param("~grid_size", 0.05)
        self.grid_size = np.array([grid_size, grid_size])
        self.vertex_map_size = (self.map_size / self.grid_size).astype(int)

        # Initialize camera parameters
        self.fov = np.radians(rospy.get_param("~angle_of_view", 30))  # 轉換為弧度
        self.ideal_range = rospy.get_param("~desired_range", 3.0)
        self.camera_variance = rospy.get_param("~camera_variance", 2)

        # # 修正：正確的代理人索引範圍
        # for i in range(self.total_agents):
        #     rospy.Subscriber(f"/{self.vehicle}_{i+1}/visualize/pose", 
        #                    Pose, self.PoseCB(i))
            
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.AgentCallback)

        rospy.Subscriber("/target", TargetInfoArray, self.TargetCallback)

        # pygame setup
        self.block_size = 2
        self.window_size = self.vertex_map_size * self.block_size
        self.screen = pygame.display.set_mode(self.window_size)
        pygame.display.set_caption("Multi-Agent Visualization")
        
        # Parameters for drawing
        self.color_pool = [(175, 100, 100), (100, 150, 100), (100, 100, 150), 
                          (255, 0, 0), (0, 255, 255), (0, 0, 255), 
                          (178, 102, 255), (255, 0, 255), (13, 125, 143)]
        
        # 預載入字體避免重複載入
        self.font = pygame.font.Font(None, 24)
    
    # def PoseCB(self, agent_id):
    #     """修正：使用agent_id避免變數名衝突"""
    #     def callback(msg):
    #         self.agent_pos[agent_id] = np.array([msg.position.x, msg.position.y])
    #         self.agent_per[agent_id] = np.array([msg.orientation.x, msg.orientation.y])
    #     return callback
    
    def AgentCallback(self, msg):
        for agent_id in range(self.total_agents):
            vehicle_name = f"{self.vehicle}_{agent_id+1}"
            if vehicle_name in msg.name:
                index = msg.name.index(vehicle_name)
                position = msg.pose[index].position
                self.agent_pos[agent_id] = np.array([position.x, position.y])
                rotation_matrix = R.from_quat([msg.pose[index].orientation.x,
                                               msg.pose[index].orientation.y,
                                                msg.pose[index].orientation.z,
                                                msg.pose[index].orientation.w])
                yaw = rotation_matrix.as_euler('zyx', degrees=False)[0]
                self.agent_per[agent_id] = np.asanyarray([np.cos(yaw), np.sin(yaw)])

    def TargetCallback(self, msg):
        """目標回調函數"""
        self.target_received = True
        self.targets = {}
        for target in msg.targets:
            pos = np.array([target.position.x, target.position.y])
            cov_matrix = np.array(target.covariance).reshape(2, 2)
            vel = np.array([target.velocity.linear.x, target.velocity.linear.y])
            requirements = list(target.required_sensor)
            
            self.targets[target.id] = {
                'pos': pos,
                'cov': cov_matrix,
                'weight': target.weight,
                'vel': vel,
                'requirements': requirements
            }

    def compute_camera_coverage_vectorized(self):
        """使用原本邏輯重寫的相機覆蓋計算"""
        # 初始化分割地圖和分數地圖
        partition_map = np.full(self.vertex_map_size, -1, dtype=int)
        score_map = np.full(self.vertex_map_size, -np.inf)
        
        # 遍歷所有代理人
        for agent_id in range(self.total_agents):
            if agent_id not in self.agent_pos:
                continue
                
            pos = self.agent_pos[agent_id]
            per = self.agent_per[agent_id]
            alpha = self.fov
            R = self.ideal_range
            sigma = self.camera_variance
            
            # 遍歷所有網格點
            for q_x in range(0, self.vertex_map_size[0], self.block_size):
                for q_y in range(0, self.vertex_map_size[1], self.block_size):
                    # 將網格索引轉換為實際座標
                    x = q_x * self.grid_size[0]
                    y = q_y * self.grid_size[1]
                    
                    # 計算距離
                    dist = np.sqrt((pos[0] - x)**2 + (pos[1] - y)**2)
                    
                    # 避免除零
                    if dist < 1e-8:
                        dist = 1e-8
                    
                    # 計算距離評分
                    q_res = np.exp(-(dist - R)**2 / (2 * sigma**2))
                    
                    # 計算方向評分（使用您原本的邏輯）
                    q_per = (1 / (1 - np.cos(alpha)))
                    q_per *= (((x - pos[0]) * per[0] + (y - pos[1]) * per[1]) / dist - np.cos(alpha))
                    
                    # 計算相機總分數
                    camera_score = q_res * q_per if q_per > 0 else -np.inf
                    
                    # 將網格座標轉換為分割地圖座標
                    map_x = q_x // self.block_size
                    map_y = q_y // self.block_size
                    
                    # 確保座標在有效範圍內
                    if (0 <= map_x < self.vertex_map_size[1] and 
                        0 <= map_y < self.vertex_map_size[0]):
                        
                        # 使用原本邏輯：只有當新分數更高時才更新
                        if camera_score > score_map[map_y, map_x]:
                            partition_map[map_y, map_x] = agent_id
                            score_map[map_y, map_x] = camera_score
        
        return partition_map

    def agent_info_update(self, agent_id):
        """agnet info update and draw"""
        if agent_id not in self.agent_pos:
            return
            
        pos = self.agent_pos[agent_id]
        per = self.agent_per[agent_id]
        
        # 轉換到螢幕座標
        screen_pos = pos / self.grid_size * self.block_size
        
        # agent position
        pygame.draw.circle(surface=self.screen,
                          color=self.color_pool[agent_id % len(self.color_pool)],
                          center=(int(screen_pos[0]), int(screen_pos[1])),
                          radius=4)
        
        # agent orientation
        orientation_end = screen_pos + (per / self.grid_size * self.ideal_range * self.block_size)
        pygame.draw.line(surface=self.screen,
                        color=self.color_pool[agent_id % len(self.color_pool)],
                        start_pos=(int(screen_pos[0]), int(screen_pos[1])),
                        end_pos=(int(orientation_end[0]), int(orientation_end[1])),
                        width=1)

        # 繪製代理人ID（含邊界檢查）
        bias = 30
        agent_name = f"{self.vehicle}_{agent_id + 1}"
        text_y = max(bias, int(screen_pos[1]) - bias)
        text_center = (int(screen_pos[0]), text_y)
        
        text = self.font.render(agent_name, True, (255, 255, 255))
        text_rect = text.get_rect(center=text_center)
        self.screen.blit(text, text_rect)
                    
    def draw_all_targets_density_combined(self):
        """方案1: 合併所有目標的密度一起繪製"""
        if not self.target_received or len(self.targets) == 0:
            return
        
        # 獲取分割地圖
        partition_map = self.compute_camera_coverage_vectorized()
        
        # 創建螢幕座標系統的網格
        screen_width = self.vertex_map_size[0] * self.block_size
        screen_height = self.vertex_map_size[1] * self.block_size
        
        x_screen = np.linspace(0, screen_width, self.vertex_map_size[0])
        y_screen = np.linspace(0, screen_height, self.vertex_map_size[1])
        X_screen, Y_screen = np.meshgrid(x_screen, y_screen)
        pos_grid = np.dstack((X_screen, Y_screen))
        
        # 計算所有目標的合併密度
        combined_density = np.zeros((self.vertex_map_size[1], self.vertex_map_size[0]))
        
        for target_id, target_data in self.targets.items():
            pos = target_data['pos']
            cov = target_data['cov']
            
            # 轉換到螢幕座標系統
            scale_factor = self.block_size / self.grid_size[0]
            screen_pos = pos * scale_factor
            screen_cov = cov * (scale_factor ** 2) * 4 ## 4是方便觀察的scale factor
            
            # 檢查有效性
            try:
                eigenvals = np.linalg.eigvals(screen_cov)
                if np.any(eigenvals <= 0):
                    screen_cov = screen_cov + np.eye(2) * 1e-6
                    
                rv = multivariate_normal(mean=screen_pos, cov=screen_cov)
                target_density = rv.pdf(pos_grid)
                
                if not (np.any(np.isnan(target_density)) or np.any(np.isinf(target_density))):
                    # 正規化單個目標的密度
                    density_min = np.min(target_density)
                    density_max = np.max(target_density)
                    if density_max > density_min:
                        normalized_target_density = (target_density - density_min) / (density_max - density_min)
                        # 加權合併（可以根據目標重要性調整權重）
                        weight = target_data.get('weight', 1.0)
                        combined_density += normalized_target_density * weight
                        
            except Exception as e:
                print(f"Error processing target {target_id}: {e}")
                continue
        
        # 正規化合併後的密度
        if np.max(combined_density) > 0:
            combined_density = combined_density / np.max(combined_density)
        
        # 繪製合併後的密度
        for x_map in range(0, self.vertex_map_size[0], 1):
            for y_map in range(0, self.vertex_map_size[1], 1):
                grid_x = x_map // self.block_size
                grid_y = y_map // self.block_size
                
                if (0 <= grid_y < partition_map.shape[0] and 
                    0 <= grid_x < partition_map.shape[1]):
                    
                    territorial_id = partition_map[grid_y, grid_x]
                    density_value = combined_density[y_map, x_map]
                    
                    screen_x = x_map * self.block_size
                    screen_y = y_map * self.block_size
                    
                    if territorial_id != -1:
                        agent_color = self.color_pool[territorial_id % len(self.color_pool)]
                        brightness_factor = 0.4 + 0.6 * density_value
                        shaded_color = (
                            max(0, min(255, int(agent_color[0] * brightness_factor))),
                            max(0, min(255, int(agent_color[1] * brightness_factor))),
                            max(0, min(255, int(agent_color[2] * brightness_factor)))
                        )
                    else:
                        base_intensity = 50
                        max_intensity = 200
                        intensity = base_intensity + (max_intensity - base_intensity) * density_value
                        intensity = max(0, min(255, int(intensity)))
                        shaded_color = (intensity, intensity, intensity)
                    
                    pygame.draw.rect(surface=self.screen,
                                color=shaded_color,
                                rect=(screen_x, screen_y, self.block_size, self.block_size))


    def Update(self):
        """主更新函數"""
        # 清空螢幕
        self.screen.fill((0, 0, 0))
    
        # 繪製所有 target 的密度
        if self.target_received:
            self.draw_all_targets_density_combined()
            # for target_id in self.targets.keys():
            #     # self.draw_target_density(target_id)
            #     print("draw target_", target_id, " density with alternative method")
            #     self.draw_target_density_alternative_screen_coords(target_id)
        
        # 繪製所有 target 的圓圈和文字
        if self.target_received:
            for target_id in self.targets.keys():
                target_data = self.targets[target_id]
                pos = target_data['pos']
                screen_pos = pos / self.grid_size * self.block_size

                # print("===============For font====================")
                # print("Drawing Target ID:", target_id)
                # print("target pos:", pos)
                # print("screen pos:", screen_pos)
                
                # 繪製目標點
                pygame.draw.circle(surface=self.screen,
                                color=(255, 255, 255),
                                center=(int(screen_pos[0]), int(screen_pos[1])),
                                radius=1)
                
                # 繪製目標ID
                text_center = (int(screen_pos[0]), int(screen_pos[1]) - 20)
                text = self.font.render(f"Target_{target_id}", True, (255, 255, 255))
                text_rect = text.get_rect(center=text_center)
                self.screen.blit(text, text_rect)
        
        # 第三步：繪製代理人
        for agent_id in range(self.total_agents):
            self.agent_info_update(agent_id)
        
        pygame.display.flip()

if __name__ == "__main__":
    rospy.init_node("visualization_node")
    pygame.init()
    
    try:
        visual = Visualization()
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # 處理pygame事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    break
            
            visual.Update()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()