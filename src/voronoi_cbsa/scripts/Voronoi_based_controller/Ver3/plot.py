#! /usr/bin/env python3
import rospy
import pygame
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
import os
import imageio
import pandas as pd

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray, Float32MultiArray, Float64MultiArray, Float64, String, Int16
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Pose
from voronoi_cbsa.msg import ExchangeData, NeighborInfoArray, TargetInfoArray, SensorArray, ValidSensors, WeightArray,ExchangeDataArray
from scipy.stats import multivariate_normal
from matplotlib.animation import FuncAnimation
import itertools

class Visualize2D():
    def __init__(self):
        self.total_agents   = rospy.get_param('/total_agents', 3)
        map_width           = rospy.get_param("/map_width", 24)
        map_height          = rospy.get_param("/map_height", 24)
        self.map_size       = np.array([map_height, map_width])
        grid_size           = rospy.get_param("/grid_size", 0.1)
        self.grid_size      = np.array([grid_size, grid_size])
        self.size           = (self.map_size/self.grid_size).astype(np.int64)
        self.sensor_pool = ['camera', 'manipulator', 'smoke_detector']
        self.agent_scores           = {}
        self.agent_sensor_scores    = {}
        self.agent_valid_sensors    = {}
        self.agent_pos              = {}
        self.agent_per              = {}
        self.start                  = 0
        self.tmp                    = [-1 for i in range(self.total_agents)]
        self.switch                 = [0 for i in range(self.total_agents)]
        self.controller             = [1 for i in range(self.total_agents)]
        self.t                      = 0
        self.tmp_score              = 0
        self.plt_utl                = []
        self.plt_time               = []
        self.target_received        = False  
        self.plot_role              = "All"      # camera, smoke_detector, manipulator, all
        self.plot_type              = "voronoi"     # voronoi,  footprint
        self.plot_link              = "cooepration" # cooperation, coordination, communication
        self.color                  = {}
        self.event_density          = {}
        self.agent_info             = {}
        self.agent_camera_info      = {}
        self.agent_manipulator_info = {}
        self.agent_smoke_detector_info = {}
        self.total_score            = []
        self.agent_score_plt        = {}
        self.frame                  = []
        self.cnt                    = 0
        self.agent_sensor_weights   = {}
        self.agent_failure          = {}
        self.vehicle                = "tb"
        #self.FetchAgentInfo()
        
        # color_pool = [(0, 255, 0), (255, 128, 0), (255,255,0), (255, 0, 0), (0,255,255), (0,0,255), (178,102,255), (255,0,255), (13, 125, 143)]
        color_pool = [(175, 100, 100), (100, 150, 100), (100,100,150), (255, 0, 0), (0,255,255), (0,0,255), (178,102,255), (255,0,255), (13, 125, 143)]
        
        rospy.Subscriber("/target", TargetInfoArray, self.TargetCallback)

        for i in range(self.total_agents):
            self.agent_sensor_scores[i]     = {}
            self.agent_sensor_weights[i]    = {}
            self.agent_score_plt[i]         = []
            self.agent_failure[i]           = False
            
            try:
                self.color[i] = color_pool[i]
            except:
                self.color[i] = list(np.random.choice(range(255),size=3))
            
            rospy.Subscriber("/"+self.vehicle+"_"+str(i+1)+"/visualize/sensor_weights", WeightArray, self.WeightCB(i))           
            rospy.Subscriber("/"+self.vehicle+"_"+str(i+1)+"/visualize/valid_sensors", ValidSensors, self.ValidSensorCB(i))
            rospy.Subscriber("/"+self.vehicle+"_"+str(i+1)+"/visualize/sensor_scores", SensorArray, self.SensorScoresCB(i))
            rospy.Subscriber("/"+self.vehicle+"_"+str(i+1)+"/visualize/total_score", Float64, self.TotalScoreCB(i))
            rospy.Subscriber("/"+self.vehicle+"_"+str(i+1)+"/visualize/pose", Pose, self.PoseCB(i))
            rospy.Subscriber("/"+self.vehicle+"_"+str(i+1)+"/failure", Int16, self.FailureCB(i))
                    
        self.window_size = self.size*4
        self.display = pygame.display.set_mode(self.window_size)
        self.display.fill((0,0,0))
        self.blockSize = int(self.window_size[0]/self.size[0])

        # 添加一個函數來翻轉x坐標
        self.flip_x = lambda x: x
        # 添加一個函數來翻轉y坐標
        self.flip_y = lambda y: self.window_size[1] - y

        # define the codec and create a video writer object
        self.cnt = 0
        self.images = []
        self.timestr = time.strftime("%Y%m%d-%H%M%S")
        self.prefix = '~/research_ws/src/voronoi_cbsa/result/'
        if not os.path.exists(self.prefix + self.timestr):
            os.makedirs(self.prefix + self.timestr)

    def flip_position(self, position):
        if isinstance(position, np.ndarray) and position.ndim > 1:
            # 處理多個點的情況
            flipped = position.copy()
            flipped[:, 0] = self.flip_x(position[:, 0])
            flipped[:, 1] = self.flip_y(position[:, 1])
            return flipped
        else:
            # 處理單個點的情況
            return np.array([self.flip_x(position[0]), self.flip_y(position[1])])
        
    def FetchAgentInfo(self):

        for agent in range(self.total_agents):
            # General Agent's Settings
            prefix = self.vehicle+"_" + str(agent) + "/Controller"
            id                      = rospy.get_param(prefix+"/id", default=0)
            camera_valid            = rospy.get_param(prefix+"/camera", default=0)
            manipulator_valid       = rospy.get_param(prefix+"manipulator", default=0)
            smoke_detector_valid    = rospy.get_param(prefix+"/smoke_detector", default=0)
            valid_sensor            = {'camera'         : camera_valid,
                                        'manipulator'    : manipulator_valid,
                                        'smoke_detector' : smoke_detector_valid}
            
            if camera_valid:
                # Camera Settings
                per_x               = rospy.get_param("~per_x", default=0)
                per_y               = rospy.get_param("~per_y", default=0)
                init_perspective    = np.array([-float(per_x), -float(per_y)])
                angle_of_view       = rospy.get_param("~angle_of_view")
                range_limit         = rospy.get_param("~desired_range")
                camera_variance     = rospy.get_param("~camera_variance", default=1)
                
                camera_info         = { 'perspective'    : init_perspective,
                                        'angle of view'  : angle_of_view,
                                        'desired range'  : range_limit,
                                        'variance'       : camera_variance}
            else:
                camera_info = None
            
            self.agent_camera_info[agent] = camera_info
            
            if manipulator_valid:
                # Manipulator Settings
                arm_length          = rospy.get_param("~arm_length", default = 1)
                approx_param        = rospy.get_param("~approx_param", default=20)
                
                manipulator_info    = { 'arm length'     : arm_length,
                                        'k'              : approx_param}
            else:
                manipulator_info = None
            
            self.agent_manipulator_info[agent] = manipulator_info

            if smoke_detector_valid:
                # Smoke Detector Settings
                smoke_variance      = rospy.get_param("~smoke_variance", default=1)
                smoke_detector_info = { 'variance'       : smoke_variance } 
            else:
                smoke_detector_info = None
                
            self.agent_smoke_detector_info[agent] = smoke_detector_info
            
            self.agent_info[agent] = {'camera_info'         : camera_info,
                                      'manipulator_info'    : manipulator_info,
                                      'smoke_detector_info' : smoke_detector_info}
            
    def TargetCallback(self, msg):
        self.target_received = True
        self.targets = []

        for target in msg.targets:
            # Fix optitrsck bias:
            #   set (0, 0)->(2.5, 2)
            #   left-down(0, 0)
            # Done in control_tb - AgentCallback(), don't need here.
            pos_x = target.position.x
            pos_y = target.position.y
            pos = np.array([pos_x, pos_y])

            # std = target.standard_deviation
            cov = target.covariance
            weight = target.weight

            vel_x = target.velocity.linear.x
            vel_y = target.velocity.linear.y
            vel = np.array([vel_x, vel_y])
            requirements = [target.required_sensor[i] for i in range(len(target.required_sensor))]
            self.targets.append([pos, cov, weight, vel, target.id, requirements])
            
    def FailureCB(self, id):
        def callback(msg):
            if msg.data == 1:
                self.agent_failure[id] = True
        
        return callback
    
    def SensorScoresCB(self, id):
        def callback(msg):
            for sensor in msg.sensors:
                self.agent_sensor_scores[id][sensor.type] = sensor.score
            
        return callback
    
    def TotalScoreCB(self, id):
        def callback(msg):
            self.agent_scores[id] = msg.data

        return callback
    
    def ValidSensorCB(self, id):
        def callback(msg):
            self.agent_valid_sensors[id] = []
            for data in msg.data:
                self.agent_valid_sensors[id].append(data) 

        return callback
    
    def PoseCB(self, id):
        def callback(msg):
            self.agent_pos[id] = np.asarray([msg.position.x, msg.position.y])
            self.agent_per[id] = np.asarray([msg.orientation.x, msg.orientation.y])

        return callback
    
    def WeightCB(self, id):
        def callback(msg):
            for sensor in msg.weights:
                self.agent_sensor_weights[id][sensor.type] = {}
            
            for sensor in msg.weights:
                self.agent_sensor_weights[id][sensor.type][sensor.event_id] = sensor.score
                
        return callback
    
    def Update(self):
        def ComputeEventDensity(map_size, target, grid_size, role):

            event = np.zeros(self.size)
            min_threshold = 1e-10

            for i in range(len(target)):
                if role in target[i][5] or role == 'All':
                    x, y = np.mgrid[0:map_size[0]:grid_size[0], 0:map_size[1]:grid_size[1]]
                    xy = np.column_stack([x.flat, y.flat])
                    mu = np.array(target[i][0])
                    # sigma = np.array([target[i][1], target[i][1]])
                    # covariance = np.diag(sigma**2)
                    covariance = np.array(target[i][1]).reshape(2,2)
                    # print("covariance matrix:\n", covariance)
                    z = multivariate_normal.pdf(xy, mean=mu, cov=covariance)
                    event += z.reshape(x.shape)
                    event = np.maximum(event, min_threshold)
                    # if np.max(event) > 0:
                    #     event = event / np.max(event) # 將event標準化到[0,1]範圍
             
            return event
        
        def ComputeSensorVoronoi(role, agents_pos, global_event):
            x_coords, y_coords = np.meshgrid(np.arange(self.size[0]), np.arange(self.size[1]), indexing='ij')

            if role == "All":
                
                total_cost = np.full(self.size, np.inf)
                sensor_voronoi = np.full(self.size, -1)

                for agent in agents_pos.keys():
                    if not self.agent_failure[agent]:
                        pos_self = agents_pos[agent]
                        grid_size = self.grid_size

                        cost = (((pos_self[0]/grid_size[0] - x_coords)**2 + (pos_self[1]/grid_size[1] - y_coords)**2))*global_event
                        sensor_voronoi = np.where(cost < total_cost, agent, sensor_voronoi)
                        total_cost = np.where(cost < total_cost, cost, total_cost)
                    
            else:
                
                total_cost = np.full(self.size, np.inf)
                sensor_voronoi = np.full(self.size, -1)
                    
                for agent in agents_pos.keys():
                    if agent in self.agent_valid_sensors.keys() and not self.agent_failure[agent]:
                        if role in self.agent_valid_sensors[agent]:
                            pos_self = agents_pos[agent]
                            grid_size = self.grid_size
                            
                            weight  = 0
                            if role in self.agent_sensor_weights[agent]:
                                for event in self.agent_sensor_weights[agent][role].keys():
                                    weight += self.agent_sensor_weights[agent][role][event]
                            
                            weight *= 50
                            if role == 'camera':
                                print(agent, ": ", weight)
                            # cost = (((pos_self[0]/grid_size[0] - x_coords)**2 + (pos_self[1]/grid_size[1] - y_coords)**2)
                            #         - weight**2)*global_event#self.ComputeCost(role, pos_self, grid_size, x_coords, y_coords)
                            cost = (((pos_self[0]/grid_size[0] - x_coords)**2 + (pos_self[1]/grid_size[1] - y_coords)**2)
                                    - weight**2)*global_event
                            sensor_voronoi = np.where(cost < total_cost, agent, sensor_voronoi)
                            total_cost = np.where(cost < total_cost, cost, total_cost)

            return sensor_voronoi.transpose()
        
        def ComputeSensorFootprint(role, agent_pos, global_event, agent_info):
            for agent in self.agent_pos.keys():
                pass

        self.display.fill((0,0,0))
        
        if self.target_received:
            event = ComputeEventDensity(self.map_size, self.targets, self.grid_size, self.plot_role)
            event_plt = ((event - event.min()) * (1/(event.max() - event.min()) * 255)).astype('uint8')
            
            if self.plot_type == 'voronoi':
                voronoi_plt = ComputeSensorVoronoi(self.plot_role, self.agent_pos.copy(), event.copy())
                for x_map, x in enumerate(range(0, self.window_size[0], self.blockSize)):
                    for y_map, y in enumerate(range(0, self.window_size[1], self.blockSize)):
                        
                        id = voronoi_plt[y_map, x_map]
                        # 翻轉畫面矩形位置
                        rect = pygame.Rect(
                            self.flip_x(x + self.blockSize) - self.blockSize, 
                            self.flip_y(y + self.blockSize) - self.blockSize, 
                            self.blockSize, 
                            self.blockSize
                        )
                        pygame.draw.rect(self.display, (event_plt[x_map,y_map],event_plt[x_map,y_map],event_plt[x_map,y_map]), rect, 1)
                        
                        if id != -1:
                            # 矩形位置已經翻轉，不需要再次翻轉
                            w = 0.5
                            color = [0, 0, 0]
                            color[0] = w*self.color[id][0] + (1-w)*event_plt[x_map,y_map]
                            color[1] = w*self.color[id][1] + (1-w)*event_plt[x_map,y_map]
                            color[2] = w*self.color[id][2] + (1-w)*event_plt[x_map,y_map]
                            pygame.draw.rect(self.display, color, rect, 1)
            
            # 文本位置翻轉
            font = pygame.font.Font('freesansbold.ttf', 30)
            text = font.render(self.plot_role, True, (255,255,255))
            textRect = text.get_rect()
            textRect.center = (self.flip_x(100), self.flip_y(50))  # 翻轉文本位置
            self.display.blit(text, textRect)
            
            for i in range(len(self.targets)):
                if self.plot_role in self.targets[i][5] or self.plot_role == 'All':
                    # 翻轉目標中心位置
                    original_center = np.array(self.targets[i][0])/self.grid_size*self.blockSize
                    center = self.flip_position(original_center)
                    
                    # offset = 5
                    # # 三角形頂點需要按照翻轉後的方向重新計算
                    # pygame.draw.polygon(self.display, (0, 0, 0), (
                    #     (center[0], center[1] + offset),           # 底部中心點變成頂部中心點
                    #     (center[0] - offset, center[1] - offset),  # 左下角變成左上角
                    #     (center[0] + offset, center[1] - offset)   # 右下角變成右上角
                    # ))
                    
                    font = pygame.font.Font('freesansbold.ttf', 15)
                    context_sensor = str(i+1) + ": " + "{"
                    for sensor in self.sensor_pool:
                        if sensor in self.targets[i][5]:
                            context_sensor += sensor + " "
                    context_sensor += "}"
                    
                    text = font.render(context_sensor, True, (0,0,0))
                    textRect = text.get_rect()
                    textRect.center = (center[0], center[1] + 30)  # 翻轉文本位置
                    self.display.blit(text, textRect)
            
            for id in self.agent_pos.keys():
                if not self.agent_failure[id]:
                    if id in self.agent_scores.keys():
                        self.agent_score_plt[id].append(self.agent_scores[id])
                    
                    # 翻轉代理位置
                    original_center = self.agent_pos[id]/self.grid_size*self.blockSize
                    center = self.flip_position(original_center)
                    
                    # 繪製ID和有效感測器
                    font = pygame.font.Font('freesansbold.ttf', 15)
                    context_sensor = "{"
                    for sensor in self.sensor_pool:
                        if id in self.agent_valid_sensors.keys():
                            if sensor in self.agent_valid_sensors[id]:
                                context_sensor += sensor + " "
                    context_sensor += "}"
                    
                    text = font.render(context_sensor, True, (0, 0, 0))
                    textRect = text.get_rect()
                    upper = center[1] + 20 if center[1] + 20 < self.window_size[1] else center[1] - 20  # 翻轉上方判斷
                    textRect.center = (center[0], upper)
                    self.display.blit(text, textRect)
                    
                    text = font.render(str(id+1), True, (0, 0, 0))
                    textRect = text.get_rect()
                    upper = center[1] + 40 if center[1] + 40 < self.window_size[1] else center[1] - 40  # 翻轉上方判斷
                    textRect.center = (center[0], upper)
                    self.display.blit(text, textRect)
                    
                    # 繪製代理位置
                    if id in self.agent_valid_sensors.keys():
                        if self.plot_role in self.agent_valid_sensors[id]:
                            width = 0
                        elif self.plot_role == "All":
                            width = 0
                        else:
                            width = 2
                        pygame.draw.circle(self.display, self.color[id], center, radius=10, width=width)
                        
                        if 'camera' in self.agent_valid_sensors[id]:
                            # 繪製代理視角，需要翻轉方向向量
                            original_pos = self.agent_pos[id]/self.grid_size*self.blockSize
                            pos = self.flip_position(original_pos)
                            
                            # 翻轉方向向量
                            original_per = self.agent_per[id]/self.grid_size*self.blockSize
                            original_per *= 0.5
                            # 方向向量需要特殊處理，翻轉後方向也要相反
                            per = np.array([original_per[0], -original_per[1]])
                            
                            pygame.draw.line(self.display, self.color[id], pos, pos + per, 2)

                            # 繪製Camera FOV(半透明扇形)
                            radius = 250  # 增加扇形半徑以便更明顯
                            
                            # 正確計算方向向量的角度
                            # 使用arctan2來獲取-180到180度的角度
                            angle_rad = np.arctan2(per[1], per[0])  # 注意：是per而不是-per[1]
                            angle_deg = np.degrees(angle_rad)
                            
                            # 扇形角度範圍
                            fov = 30  # 視場角度(Field of View)
                            start_angle = angle_deg - fov/2
                            end_angle = angle_deg + fov/2
                            
                            # 創建半透明扇形
                            color = self.color[id]
                            alpha = 100  # 透明度值(0-255)
                            
                            # 計算扇形的頂點
                            points = [pos]
                            for angle in range(int(start_angle), int(end_angle) + 1, 1):
                                rad = np.radians(angle)
                                x = pos[0] + radius * np.cos(rad)
                                y = pos[1] + radius * np.sin(rad)
                                points.append((x, y))
                            
                            # 創建適當大小的臨時表面
                            temp_surface = pygame.Surface(self.window_size, pygame.SRCALPHA)
                            # 繪製扇形到臨時表面
                            pygame.draw.polygon(temp_surface, (*color, alpha), points)
                            self.display.blit(temp_surface, (0, 0))

                            
                else:
                    # 繪製失效代理位置
                    original_center = self.agent_pos[id]/self.grid_size*self.blockSize
                    center = self.flip_position(original_center)
                    
                    if id in self.agent_valid_sensors.keys():
                        if self.plot_role in self.agent_valid_sensors[id]:
                            width = 0
                        elif self.plot_role == "All":
                            width = 0
                        else:
                            width = 2
                        pygame.draw.circle(self.display, (125, 125, 125), center, radius=10, width=width)
            
            # if score_ready:
            #     font = pygame.font.Font('freesansbold.ttf', 32)
            #     text = font.render(str(np.round(total_score, 3)), True, (255,255,255))
            #     textRect = text.get_rect()
            #     textRect.center = (100, 80)
            #     self.display.blit(text, textRect)

            #     self.total_score.append(total_score)
            #     self.frame.append(self.cnt)
            #     self.cnt += 1
                
        pygame.display.flip()
        
        # transform the pixels to the format used by open-cv
        pixels = cv2.rotate(pygame.surfarray.pixels3d(self.display), cv2.ROTATE_90_CLOCKWISE)
        pixels = cv2.flip(pixels, 1)

        self.images.append(pixels)
        #self.plot()

    def plot(self):
        plt.plot(self.frame, self.total_score, color = 'b')
        plt.title("Total score")
        plt.xlabel('Frame id')
        plt.ylabel('Score')
        
        plt.draw()  
        plt.pause(0.01)  
    
    #def plot_graph(self):
        
        
    def End(self):
        imageio.mimsave(self.prefix+"/8/coop/balance"+'.gif', self.images)
 
def KillCB(msg):
    global kill
    if msg.data == 1:
        kill = True  
         
if __name__=="__main__":
    global kill
    kill = False
    
    rospy.init_node('visualizer', anonymous=False, disable_signals=True)
    
    total_agents = rospy.get_param('/total_agents', '3')
    rospy.Subscriber("/kill", Int16, KillCB)
    
    pygame.init()

    visualizer = Visualize2D()
    while True:
        if kill:
            visualizer.End()
            break
        
        visualizer.Update()
            
    pygame.quit()   

    # plt_dict = {'frame_id': visualizer.frame,
    #             'total score': visualizer.total_score}
    
    # for id in range(total_agents):
    #     if len(visualizer.agent_score_plt[id]) > 0:
    #         plt_dict[str(id)+"'s score"] = visualizer.agent_score_plt[id]
        
    # df = pd.DataFrame.from_dict(plt_dict) 
    # df.to_csv (r'~/research_ws/src/voronoi_cbsa/result/non-coop-3.csv', index=False, header=True)