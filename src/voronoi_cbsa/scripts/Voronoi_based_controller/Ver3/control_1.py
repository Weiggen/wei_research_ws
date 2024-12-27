#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Point, PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL
from voronoi_cbsa.msg import ExchangeData, ExchangeDataArray, TargetInfoArray, SensorArray, Sensor, ValidSensors, WeightArray, Weight, densityGradient
from std_msgs.msg import Int16, Int32, Bool, Float32MultiArray, Int16MultiArray, Float32, Float64, Float64MultiArray, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates, LinkStates
from qpsolvers import solve_qp
from cvxopt import matrix, solvers

import numpy as np
import pandas as pd
import os
from time import time, sleep
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
import itertools
import math
import _thread
import tf.transformations as tf

class CMD:
    def __init__(self, id):
        self.arm_cmd = False
        self.mode_cmd = 0
        self.ID = id
        self.curr_mode = 0
        self.offb_curr_mode = ""
        self.vision_tracking = False
        
        self.arming_client = rospy.ServiceProxy("/iris_"+str(id)+"/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/iris_"+str(id)+"/mavros/set_mode", SetMode)
        self.takeoff_client = rospy.ServiceProxy("/iris_"+str(id)+"/mavros/cmd/takeoff", CommandTOL)
        
        self.arm_cmd_sub = rospy.Subscriber("/formation/all_uav_arm", Bool, self.arm_cmd_cb)
        self.mode_cmd_sub = rospy.Subscriber("/formation/all_uav_mode", Int32, self.mode_cmd_cb)
      
    def arm_cmd_cb(self, msg):
        self.arm_cmd = msg.data
        self.setArm(self.arm_cmd)
        
    def mode_cmd_cb(self, msg):
        self.mode_cmd = msg.data
        self.setMode(self.mode_cmd)
        
    def setArm(self, arm_CMD):
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = arm_CMD
        if self.arming_client(arm_cmd).success:
            rospy.loginfo(f"UAV_{self.ID} armed switch successfully")
        else:
            rospy.logwarn(f"UAV_{self.ID} failed to arm")
            
    def setMode(self, mode_CMD):
        offb_set_mode = SetModeRequest()
        if mode_CMD == 0:
            offb_set_mode.custom_mode = "STABILIZED"
        elif mode_CMD == 1:
            offb_set_mode.custom_mode = "AUTO.TAKEOFF"
        elif mode_CMD == 2:
            offb_set_mode.custom_mode = "AUTO.LAND"
        elif mode_CMD == 3:
            offb_set_mode.custom_mode = "OFFBOARD"
        elif mode_CMD == 5:
            if not self.vision_tracking:
                rospy.loginfo("Tracking by vision")
                self.vision_tracking = True
            else:
                rospy.loginfo("Stop vision tracking")
                self.vision_tracking = False
                
        if offb_set_mode.custom_mode != self.offb_curr_mode:
            if self.set_mode_client(offb_set_mode).mode_sent:
                rospy.loginfo(f"UAV_{self.ID} mode switched to {offb_set_mode.custom_mode}")
                self.offb_curr_mode = offb_set_mode.custom_mode
                if offb_set_mode.custom_mode == "AUTO.TAKEOFF":
                    rospy.sleep(5)
            else:
                rospy.loginfo(f"UAV_{self.ID} failed to switch mode")

    def mode_CMD(self):
        return self.mode_cmd

class PTZCamera():
    def __init__(self, map_size, grid_size, general_properties,
                 coop, balance, strength, camera_properties=None, 
                 manipulator_properties=None, smoke_detector_properties=None, K_p=0.8, K_v=0.5, step = 0.1):

        self.K_p            = K_p
        self.K_v            = K_v
        self.u_p            = np.array([0., 0.])
        self.u_v            = np.array([0., 0.])
        self.step           = step
        self.cooperation    = coop 
        self.sensor_balance = balance
        self.w_coop         = strength
        self.safe_distance  = 1
        self.avoid_weight   = 0.05
        
        # Setting up environment parameters
        self.total_agents       = rospy.get_param('/total_agents', 1)
        self.visualize_option   = rospy.get_param('/visualize_option', default = -1)
        self.grid_size          = grid_size
        self.map_size           = map_size
        self.size               = (int(map_size[0]/grid_size[0]), int(map_size[1]/grid_size[1]))
        
        # Setting up agent's general parameters
        self.id                 = general_properties['id']
        self.pos                = general_properties['position']
        self.pos_z              = Float32
        self.valid_sensors      = general_properties['valid_sensor']
        self.max_speed          = general_properties['max_speed']
        self.sensor_qualities   = {}
        self.agent_ready        = False

        if self.valid_sensors['camera']:
            # Camera property
            self.perspective        = camera_properties['perspective']/self.Norm(camera_properties['perspective'])
            self.camera_range       = camera_properties['desired range']
            self.angle_of_view      = camera_properties['angle of view']
            self.camera_variance    = camera_properties['variance']
        else:
            # Camera property
            self.perspective        = np.array([1.,0.])
            self.camera_range       = 0
            self.angle_of_view      = 0
            self.camera_variance    = 0
            
        if self.valid_sensors['manipulator']:
            # Manipulator property
            self.operation_range    = manipulator_properties['arm length']
            self.approx_param       = manipulator_properties['k']
        else:
            # Manipulator property
            self.operation_range    = 0
            self.approx_param       = 0
            
        if self.valid_sensors['smoke_detector']:
            # Smoke Detector property
            self.smoke_variance = smoke_detector_properties['variance']
        else:
            self.smoke_variance = 0
            
        self.target_ready        = False
        self.targets                = {}
        self.target_buffer          = {}
            
        self.event_density          = {}
        self.event_density_buffer   = {}

        # self.event_density_gradient = []
        self.event_density_gradient = [np.zeros(self.size), np.zeros(self.size)]
        
        self.neighbors              = []
        self.neighbors_buffer       = {}
        
        self.sensor_graph           = {}
        self.sensor_voronoi         = {}
        self.sensor_weight          = {}
        
        self.total_score = 0
        for sensor in self.valid_sensors.keys():
            self.sensor_graph[sensor]       = []
            self.sensor_voronoi[sensor]     = {}  
            self.sensor_qualities[sensor]   = {}  
            self.sensor_weight[sensor]      = {} 
            
        self.RosInit()
        self.debug = self.DEBUGTOOL(id = self.id)
        
    def RosInit(self):

        rospy.Subscriber("local/neighbor_info", ExchangeDataArray, self.NeighborCallback)
        rospy.Subscriber("local/target", TargetInfoArray, self.TargetCallback)      
        # rospy.Subscriber("/target", TargetInfoArray, self.TargetCallback)  
        # rospy.Subscriber("/iris_"+str(self.id)+"/mavros/local_position/pose", PoseStamped, self.AgentPosCallback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.AgentPosCallback)
        rospy.Subscriber("/iris_"+str(self.id)+"/densityGradient", densityGradient, self.DensityGradientCallback)
        rospy.Subscriber("/iris_"+str(self.id)+"/heading_cmd", Float64MultiArray, self.HeadingCmdCallback)

        self.pub_pos                = rospy.Publisher("local/position", Point, queue_size=10)
        self.pub_exchange_data      = rospy.Publisher("local/exchange_data",ExchangeData, queue_size=10)
        self.pub_utility            = rospy.Publisher("/iris_"+str(self.id)+"/utility", Float64, queue_size=10)

        self.pub_vel_cmd            = rospy.Publisher("/iris_"+str(self.id)+"/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)       
        self.pub_pos_cmd            = rospy.Publisher("/iris_"+str(self.id)+"/mavros/setpoint_position/local", PoseStamped, queue_size=10)       
        self.pub_heading_cmd        = rospy.Publisher("/iris_"+str(self.id)+"/heading_cmd", Float64MultiArray, queue_size=10)

        self.pub_sensor_weight      = rospy.Publisher("visualize/sensor_weights", WeightArray, queue_size=10)
        self.pub_total_score        = rospy.Publisher("visualize/total_score", Float64, queue_size=10)
        self.pub_sensor_scores      = rospy.Publisher("visualize/sensor_scores", SensorArray, queue_size=10)
        self.pub_pose               = rospy.Publisher("visualize/pose", Pose, queue_size=10)
        self.pub_valid_sensors      = rospy.Publisher("visualize/valid_sensors", ValidSensors, queue_size=10)
        
        self.pub_sensor_graphs      = {}
        
        for sensor in self.valid_sensors.keys():
            if self.valid_sensors[sensor]:
                self.pub_sensor_graphs[sensor] = rospy.Publisher("visualize/"+sensor+"_neighbor", Int16MultiArray, queue_size = 10)
        
    def HeadingCmdCallback(self, msg):
        if len(msg.data) == 2:
            self.u_v = np.array(msg.data)

    def NeighborCallback(self, msg):
        self.neighbors_buffer = {}
        
        for neighbor in msg.data:

            pos_x   = neighbor.position.x
            pos_y   = neighbor.position.y
            pos     = np.array([pos_x, pos_y])
            role    = {}
            weights = {}
            sensor_scores = {}
            
            for sensor in neighbor.role.sensors:
                role[sensor.type] = sensor.score

            for weight in neighbor.weights.weights:
                weights[weight.type] = {}
                
            for weight in neighbor.weights.weights:
                weights[weight.type][weight.event_id] = weight.score
            
            for score in neighbor.sensor_scores.weights:
                sensor_scores[score.type] = {}
            
            for score in neighbor.sensor_scores.weights:
                sensor_scores[score.type][score.event_id] = score.score

            vel_x = neighbor.velocity.x
            vel_y = neighbor.velocity.y
            vel = np.array([vel_x, vel_y])
                    
            self.neighbors_buffer[neighbor.id] = {"position":   pos, "role": role, "operation_range": neighbor.operation_range,
                                                  "approx_param": neighbor.approx_param, "smoke_variance": neighbor.smoke_variance,
                                                  "camera_range": neighbor.camera_range, "angle_of_view": neighbor.angle_of_view,
                                                  "camera_variance": neighbor.camera_variance, "weights": weights, "sensor_scores": sensor_scores,
                                                  "velocity": vel}
    
    def TargetCallback(self, msg):
        self.target_ready = True
        for target in msg.targets:

            pos_x   = target.position.x
            pos_y   = target.position.y
            pos     = np.array([pos_x, pos_y])
            height  = target.height

#           std     = target.standard_deviation
            cov     = target.covariance
            weight  = target.weight

            vel_x   = target.velocity.linear.x
            vel_y   = target.velocity.linear.y
            vel     = np.array([vel_x, vel_y])
            
            requirements = [target.required_sensor[i] for i in range(len(target.required_sensor))]

            self.target_buffer[target.id] = [pos, cov, weight, vel, target.id, requirements, height]
                
    def AgentPosCallback(self, msg):
        # self.agent_ready = True
        # self.pos = np.array([msg.pose.position.x, msg.pose.position.y])
        # self.pos_z = msg.pose.position.z
        # q_current = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        # euler = tf.euler_from_quaternion(q_current)
        # self.yaw = euler[2] # rad
        # self.perspective = [math.cos(self.yaw), math.sin(self.yaw)] # [v_x, v_y]
        # # print("heading vector_"+str(self.id)+": \n{}\n".format(self.perspective))
        self.agent_ready = True
        index = msg.name.index("iris"+str(self.id))
        pose = msg.pose[index]
        self.pos = np.array([pose.position.x, pose.position.y])
        self.pos_z = pose.position.z
        q_current = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        euler = tf.euler_from_quaternion(q_current)
        self.yaw = euler[2] # rad
        self.perspective = [math.cos(self.yaw), math.sin(self.yaw)] # [v_x, v_y]

    def DensityGradientCallback(self, msg):
        self.agent_ready = True
        self.event_density_gradient = [np.zeros(self.size), np.zeros(self.size)]
        gradient_x_array = np.array(msg.gradient_x).reshape(self.size[0], self.size[1])
        gradient_y_array = np.array(msg.gradient_y).reshape(self.size[0], self.size[1])
        gradient_x_array = np.nan_to_num(gradient_x_array, nan=0.0)
        gradient_y_array = np.nan_to_num(gradient_y_array, nan=0.0)
        self.event_density_gradient[0] += gradient_x_array
        self.event_density_gradient[1] += gradient_y_array
        # rospy.loginfo("Size of self.event_density_gradient[0]: {}".format(self.event_density_gradient[0].shape))
        # rospy.loginfo("Size of self.event_density_gradient[1]: {}".format(self.event_density_gradient[1].shape))
        # rospy.loginfo("msg.gradient_x: \n{}".format(np.array(msg.gradient_x)[:3]))
        # rospy.loginfo("self.event_density_gradient[0]: \n{}".format(self.event_density_gradient[0][:3, :3]))

    def PublishInfo(self):
        # publish position
        pos     = Point()
        pos.x   = self.pos[0]
        pos.y   = self.pos[1]
        self.pub_pos.publish(pos)

        # Publish total score
        total_score  = Float64()

        total_score.data = self.total_score
        self.pub_total_score.publish(total_score)
                
        #publish exchange data
        data = ExchangeData()

        data.id = self.id
        data.position.x = self.pos[0]
        data.position.y = self.pos[1]
        data.operation_range = self.operation_range
        
        sensor_arr = SensorArray()
        sensor_arr.sensors = []
        weight_arr = WeightArray()
        weight_arr.weights = []
        scores_arr = WeightArray()
        scores_arr.weights = []
        
        for i in self.valid_sensors.keys():
            sensor = Sensor()
            sensor.type = i
            sensor.score = self.valid_sensors[i]
            sensor_arr.sensors.append(sensor)

            for event in self.targets.keys():
                if event in self.sensor_weight[i].keys():
                    weight          = Weight()
                    weight.event_id = event
                    weight.type     = i
                    weight.score    = self.sensor_weight[i][event]
                    weight_arr.weights.append(weight)
                
                else:
                    weight          = Weight()
                    weight.event_id = event
                    weight.type     = i
                    weight.score    = 0
                    weight_arr.weights.append(weight)
                
                weight          = Weight()
                weight.event_id = event
                weight.type     = i
                weight.score    = self.sensor_scores[i][event]
                scores_arr.weights.append(weight)
                
                
        data.operation_range    = self.operation_range
        data.approx_param       = self.approx_param
        data.smoke_variance     = self.smoke_variance
        data.camera_range       = self.camera_range
        data.angle_of_view      = self.angle_of_view
        data.camera_variance    = self.camera_variance
        
        data.sensor_scores  = scores_arr
        data.weights        = weight_arr
        data.role           = sensor_arr
   
        self.pub_exchange_data.publish(data)
        self.pub_sensor_weight.publish(weight_arr)
        
        # publish valid sensors
        valid_sensor = ValidSensors()
        valid_sensor.data = []
        for sensor in self.valid_sensors.keys():
            if self.valid_sensors[sensor]:
                valid_sensor.data.append(sensor)
        self.pub_valid_sensors.publish(valid_sensor)
        
        # publish pose
        pose = Pose()
        pose.position.x = self.pos[0]
        pose.position.y = self.pos[1]
        pose.orientation.x = self.perspective[0]
        pose.orientation.y = self.perspective[1]
        
        self.pub_pose.publish(pose)

        # publish sensor graphs
        for sensor in self.valid_sensors.keys():
            if self.valid_sensors[sensor] and sensor in self.neighbors.keys():
                neighbor = [i for i in self.neighbors[sensor]]
                self.pub_sensor_graphs[sensor].publish(neighbor)
        
    # Find the agent's voronoi neighbors
    def ComputeNeighbors(self, role):
        def CountNeighborRole(neighbors, role):
            neighbor = []
            if role == -1:
                for n in neighbors.keys():
                    neighbor.append(n)
            
            else:
                for n in neighbors.keys():
                    if neighbors[n]["role"][role] > 0:
                        neighbor.append(n)

            return neighbor
        
        valid_neighbor = CountNeighborRole(self.neighbors, role)
        
        if len(valid_neighbor) >= 4:

            keys = [self.id]
            for key in valid_neighbor:
                keys.append(key)

            idx_map = {}
            idx_list = []

            for i, key in enumerate(keys):
                idx_map[key] = i
                idx_list.append(key)

            points = [0 for i in keys]
            points[0] = self.pos/self.grid_size

            for member in valid_neighbor:
                pos = np.asarray([self.neighbors[member]["position"][0], self.neighbors[member]["position"][1]])
                points[idx_map[member]] = pos/self.grid_size

            points = np.asarray(points)
            tri = Delaunay(points)

            ids = []
            for simplex in tri.simplices:
                if idx_map[self.id] in simplex:
                    for id in simplex:
                        ids.append(id)

            neighbors = []
            for member in valid_neighbor:
                if idx_map[member] in ids:

                    neighbors.append(member)
            
        elif len(valid_neighbor) >= 0:

            neighbors = []
            for member in valid_neighbor:
                neighbors.append(member)
            
        else:
            neighbors = []            
               
        return neighbors
                        
    def Update(self):
        if self.target_ready and self.agent_ready:
            self.neighbors          = self.neighbors_buffer.copy()                      
            self.targets            = self.target_buffer.copy()   
                
            self.total_score    = 0
            self.sensor_scores = {}
            for i in self.valid_sensors.keys():
                self.sensor_scores[i] = {}
                for event in self.targets.keys():
                    self.sensor_scores[i][event]  = 0.
            
            for target in self.targets.keys():
                self.event_density[target]    = self.ComputeEventDensity(target = self.targets[target])
            
            for sensor in self.valid_sensors.keys():
                self.sensor_graph[sensor]       = []
                self.sensor_voronoi[sensor]     = {}   
                self.sensor_qualities[sensor]   = {}  
                self.sensor_weight[sensor]      = {}       
            
            self.ComputeSensorWeight()
            
            self.sensor_graph[-1] = self.ComputeNeighbors(role = -1)
            
            for role in self.valid_sensors.keys():
                self.sensor_graph[role] = self.ComputeNeighbors(role=role)
                for event in self.targets.keys():
                    self.sensor_voronoi[role][event] = np.zeros(self.size, dtype=object)
                    self.UpdateSensorVoronoi(role = role, event = event)
                        
            u_p, u_v = self.ComputeControlSignal()
            self.UpdatePosition(u_p)

            if self.valid_sensors['camera']:
                self.UpdatePerspective(u_v)
                array_msg = Float64MultiArray()
                array_msg.data = u_v.tolist()
                self.pub_heading_cmd.publish(array_msg)

            twistStamped_msg = TwistStamped()
            twistStamped_msg.header.stamp = rospy.Time.now()
            twistStamped_msg.twist.linear.x = self.u_p[0]
            twistStamped_msg.twist.linear.y = self.u_p[1]
            twistStamped_msg.twist.linear.z = self.u_h
            twistStamped_msg.twist.angular.z = self.yaw_rate
            self.pub_vel_cmd.publish(twistStamped_msg)
            
            # if self.valid_sensors['camera']:
            #     self.UpdatePerspective(u_v)
            
            self.ComputeUtility()    
            self.PublishInfo()
            
    def UpdatePosition(self, u_p):
        # Maximum Speed restriction
        k = 1.2
        u_p = k*u_p

        for role in self.valid_sensors.keys():
            for event in self.targets.keys():
                u_p = self.qp(role = role, event = event, u_des = u_p)

        if np.linalg.norm(u_p) > self.max_speed:
            u_p = self.max_speed*(u_p/np.linalg.norm(u_p))

        if self.pos[0] < 0 or self.pos[0] > self.map_size[0]:
            self.u_p[0] = 0
        else :
            self.u_p[0] = u_p[0]

        if self.pos[1] < 0 or self.pos[1] > self.map_size[1]:
            self.u_p[1] = 0
        else :
            self.u_p[1] = u_p[1]

        # Height
        target_heights = {}
        for target in self.targets.keys():
            target_heights[target] = self.targets[target][6]
        if target_heights:
            hightest = max(target_heights.values())
        k_h = 1
        tolerance = 1
        ideal_z = 2.5 + hightest
        if self.pos_z < ideal_z - tolerance :
            self.u_h = k_h*(ideal_z - self.pos_z)
        elif self.pos_z > ideal_z + tolerance:
            self.u_h = k_h*(ideal_z - self.pos_z)
        else :
            self.u_h = 0

        ###########################################################
        # u_p = self.max_speed*(u_p/np.linalg.norm(u_p))
        # self.u_p = u_p

        # u_avoid = np.array([0., 0.])
          
        # self.pos += self.K_p * ((1 - self.avoid_weight)*u_p + self.avoid_weight*u_avoid) * self.step if not np.isnan(u_p)[0] else self.pos
        
        # if self.pos[0] < 0:
        #     self.pos[0] = 0
        # elif self.pos[0] > self.map_size[0]:
        #     self.pos[0] = self.map_size[0]
            
        # if self.pos[1] < 0:
        #     self.pos[1] = 0
        # elif self.pos[1] > self.map_size[1]:
        #     self.pos[1] = self.map_size[1]
                
    def UpdatePerspective(self, u_v):
        
        # yaw_c = math.atan2(self.perspective[1], self.perspective[0])
        # print("yaw_current"+str(self.id)+":{}\n".format(yaw_c))
        yaw_c = self.yaw
        
        try:
            turning = math.acos((u_v @ self.perspective.T)/np.linalg.norm(u_v))
            if  turning > 15/180*np.pi:
                u_v *= (15/180*np.pi)/turning
        except:
            # u_v = np.array([0., 0.])
            u_v = self.u_v
            turning = 0.0
            
        self.perspective += self.K_v*u_v*self.step
        self.perspective /= self.Norm(self.perspective)

        if self.pos[0] + self.perspective[0] < 0 or self.pos[0] + self.perspective[0] > 24:
           self.perspective[0] *= -1  
        
        if self.pos[1] + self.perspective[1] < 0 or self.pos[1] + self.perspective[1] > 24:
           self.perspective[1] *= -1 

        # yaw_d = math.atan2(self.perspective[1], self.perspective[0])
        u_yaw = -math.sin(yaw_c)*u_v[0]+math.cos(yaw_c)*u_v[1]
        k_yaw = 0.1415
        self.yaw_rate = k_yaw*u_yaw
           
    def UpdateSensorVoronoi(self, role, event):
        x_coords, y_coords = np.meshgrid(np.arange(self.size[0]), np.arange(self.size[1]), indexing='ij')

        global_event = np.zeros(self.size)
        target = self.targets[event]
        if role in target[5]:
            global_event = self.event_density[event]
            
            if self.valid_sensors[role]:
                pos_self = self.pos
                grid_size = self.grid_size
                
                total_cost = (((pos_self[0]/grid_size[0] - x_coords)**2 + (pos_self[1]/grid_size[1] - y_coords)**2) - (100*self.sensor_weight[role][event])**2)*global_event
                sensor_voronoi = np.full(self.size, self.id, dtype=object)
                
            else:
                total_cost = np.full(self.size, np.inf, dtype=object)
                sensor_voronoi = np.full(self.size, -1)
                
            for neighbor in self.sensor_graph[role]:
                pos_self = self.neighbors[neighbor]["position"]
                grid_size = self.grid_size

                cost = (((pos_self[0]/grid_size[0] - x_coords)**2 + (pos_self[1]/grid_size[1] - y_coords)**2) - (100*self.neighbors[neighbor]["weights"][role][event])**2)*global_event#self.ComputeCost(role, pos_self, grid_size, x_coords, y_coords)
                sensor_voronoi = np.where(cost < total_cost, neighbor, sensor_voronoi)
                total_cost = np.where(cost < total_cost, cost, total_cost)

            self.sensor_voronoi[role][event] = sensor_voronoi

    def qp(self, role, event, u_des):
        # CBF
        alpha = 1.
        d_min = 0.15

        P = matrix(np.eye(2))
        q = matrix(-u_des)
        rel_pos = np.zeros_like(self.pos)
        
        target = self.targets[event]

        G_ = []
        h_ = []

        if len(self.neighbors) > 0:
            if role in target[5]:
                for neighbor in self.sensor_graph[role]:
                    rel_pos = self.pos - self.neighbors[neighbor]["position"]
                    d = np.linalg.norm(rel_pos)
                    h_c = d**2-d_min**2
                    G = matrix(-2.0*rel_pos.reshape(1, 2))
                    h = matrix(alpha*h_c - 2.0*np.dot(rel_pos, self.neighbors[neighbor]["velocity"]))
                    G_.append(G)
                    h_.append(h)

        if len(G_) > 0:
            G = matrix(np.vstack(G_))
            h = matrix(np.vstack(h_))
            try:
                solution = solvers.qp(P, q, G, h)
                if solution['status'] == 'optimal':
                    # print("CBF triggered.")
                    return np.array(solution['x']).flatten()
                else:
                    return u_des
            except:
                return u_des
        else:
            return u_des

    def ComputeSensorWeight(self):
        if self.sensor_balance:
            for event in self.targets.keys():
                for sensor in self.valid_sensors.keys():
                    single_sensor_sum = 0
                    if self.valid_sensors[sensor] and sensor in self.targets[event][5]:
                        for neighbor in self.neighbors.values():  
                            if neighbor['role'][sensor] > 0:
                                single_sensor_sum += neighbor['sensor_scores'][sensor][event]
                                
                        self.sensor_weight[sensor][event] = (1 + self.sensor_scores[sensor][event])/(1 + self.sensor_scores[sensor][event] + single_sensor_sum)
                    
                    else: 
                        self.sensor_weight[sensor][event] = 0

            total = 0
            for event in self.targets.keys():
                for sensor in self.valid_sensors.keys():
                    if self.valid_sensors[sensor]:
                        total += self.sensor_weight[sensor][event]
            
            for event in self.targets.keys():
                for sensor in self.valid_sensors.keys():
                    if self.valid_sensors[sensor]:
                        self.sensor_weight[sensor][event] = self.sensor_weight[sensor][event]/total if total > 0 else 1
            
        else:
            cnt = 0
            for event in self.targets.keys():
                for sensor in self.valid_sensors.keys():
                    if self.valid_sensors[sensor] and sensor in self.targets[event][5]:
                        cnt += 1
            
            for event in self.targets.keys():
                for sensor in self.valid_sensors.keys():
                    if self.valid_sensors[sensor] and sensor in self.targets[event][5]:
                        self.sensor_weight[sensor][event] = 1/cnt
                    else:
                        self.sensor_weight[sensor][event] = 0
                                    
    def ComputeUtility(self):
        
        for event in self.targets.keys():
            for role in self.valid_sensors.keys():
                tmp = np.zeros(self.size, dtype=np.float64)
                quality = np.zeros(self.size, dtype=np.float64)
                if self.valid_sensors[role] and role in self.targets[event][5]:
                    quality = self.ComputeSelfQuality(role=role, event=event)
                    tmp = quality
                    
                    for k in self.valid_sensors.keys():
                        if k in self.targets[event][5] and k != role:
                            coop_quality = self.ComputeCooperateQuality(coop_role = role, coop_quality = quality, role=k, event = event, eval = True)

                            tmp *= (1 + self.w_coop*coop_quality)

                    tmp *= self.event_density[event]*self.grid_size[0]
                    # print("888888888: {}".format(self.event_density[event].shape)) # (240, 240)

                    self.total_score += self.sensor_weight[role][event]*np.sum(tmp)
                self.sensor_scores[role][event] = np.sum(quality)
        self.pub_utility.publish(self.total_score)
                    
    def ComputeControlSignal(self):
        u_p = np.array([0., 0.])  
        u_v = np.array([0., 0.])
        # k_1 = 0.2
        # k_2 = 0.0000000001
        k_1 = .255
        k_2 = .00000000125
        total_gradient = [np.zeros(self.size), np.zeros(self.size)]
        sensor_gradient = [np.zeros(self.size), np.zeros(self.size)]
        event_gradient = [np.zeros(self.size), np.zeros(self.size)]
        f = np.zeros(self.size)
        
        for event in self.targets.keys():
            for role in self.valid_sensors.keys():
                total_gradient[0] = np.zeros(self.size)
                total_gradient[1] = np.zeros(self.size)
                
                if self.valid_sensors[role] and role in self.targets[event][5]:
                    
                    sensor_gradient[0] = self.ComputeGradient(role, event, 'x') # f'
                    sensor_gradient[1] = self.ComputeGradient(role, event, 'y') # f'
                    event_gradient[0] = self.ComputeDensityGradient('camera', event, 'x') # the partial derivate of L_\phi
                    event_gradient[1] = self.ComputeDensityGradient('camera', event, 'y')
                                        
                    perspective_gradient_x = self.ComputeGradient('camera', event, 'perspective_x')
                    perspective_gradient_y = self.ComputeGradient('camera', event, 'perspective_y')
                    
                    if self.cooperation == True:
                        for k in self.valid_sensors.keys():
                            if k in self.targets[event][5] and k != role:
                                coop_quality = self.ComputeCooperateQuality(coop_role = role, role=k, event = event)
                                sensor_gradient[0] *= (1 + self.w_coop*coop_quality)
                                sensor_gradient[1] *= (1 + self.w_coop*coop_quality)

                                if role == 'camera':
                                    perspective_gradient_x *= (1 + self.w_coop*coop_quality)
                                    perspective_gradient_y *= (1 + self.w_coop*coop_quality)
                    
                    if role == 'camera':
                        u_v[0] += self.sensor_weight[role][event]*np.sum(perspective_gradient_x * self.event_density[event]) # ctrl signal for heading
                        u_v[1] += self.sensor_weight[role][event]*np.sum(perspective_gradient_y * self.event_density[event]) # ctrl signal for heading
                                
                    total_gradient[0] = sensor_gradient[0]
                    total_gradient[1] = sensor_gradient[1]
                    
                    if self.cooperation == True:
                        f = self.ComputeSelfQuality(role=role, event = event)
                        for i in self.valid_sensors.keys():
                            if self.valid_sensors[i] and i != role and k in self.targets[event][5]:
                                sensor_gradient[0] = self.w_coop*self.ComputeGradient(i, event, 'x')*(f)
                                sensor_gradient[1] = self.w_coop*self.ComputeGradient(i, event, 'y')*(f)
                                
                                for k in self.valid_sensors.keys():
                                    if k in self.targets[event][5] and k != i and k != role:
                                        coop_quality = self.ComputeCooperateQuality(coop_role = role, role=k, event = event).astype('int64')
                                        sensor_gradient[0] *= (1 + self.w_coop*coop_quality)
                                        sensor_gradient[1] *= (1 + self.w_coop*coop_quality)
                        
                                total_gradient[0] += sensor_gradient[0]
                                total_gradient[1] += sensor_gradient[1]

                total_gradient[0] *= self.event_density[event]
                total_gradient[1] *= self.event_density[event]
                event_gradient[0] *= f
                event_gradient[1] *= f
                # print("sensor_gradient[0]"+str(self.id)+": \n {}".format(sensor_gradient[0]))
                # print("self.event_density[event]"+str(self.id)+": \n {}".format(self.event_density[event]))
                
                tmp_x = k_1*self.sensor_weight[role][event]*np.sum(total_gradient[0])
                tmp_y = k_1*self.sensor_weight[role][event]*np.sum(total_gradient[1])
                # print("self.sensor_weight[role][event]"+str(self.id)+": \n {}".format(self.sensor_weight[role][event]))
                # print("np.sum(total_gradient[0])"+str(self.id)+": \n {}".format(np.sum(total_gradient[0])))

                tmp_x_2 = k_2*self.sensor_weight[role][event]*np.sum(event_gradient[0])
                tmp_y_2 = k_2*self.sensor_weight[role][event]*np.sum(event_gradient[1])

                u_p[0] = u_p[0] + (tmp_x if not np.isnan(tmp_x) else 0) + (tmp_x_2 if not np.isnan(tmp_x_2) else 0)
                u_p[1] = u_p[1] + (tmp_y if not np.isnan(tmp_y) else 0) + (tmp_y_2 if not np.isnan(tmp_y_2) else 0)
                # u_p[0] = u_p[0] + tmp_x + tmp_x_2
                # u_p[1] = u_p[1] + tmp_y + tmp_y_2
                # print("u_p[0]"+str(self.id)+": {}\n".format(u_p[0]))
                # print("u_p[1]"+str(self.id)+": {}\n".format(u_p[1]))
                # print("tmp_x"+str(self.id)+": {}\n".format(tmp_x))
                # print("tmp_y"+str(self.id)+": {}\n".format(tmp_y))
                # print("tmp_x_2"+str(self.id)+": {}\n".format(tmp_x_2))
                # print("tmp_y_2"+str(self.id)+": {}\n".format(tmp_y_2))
        
        return u_p, u_v
              
    def ComputeGradient(self, role, event, type):
        x_coords, y_coords = np.meshgrid(np.arange(self.size[0]), np.arange(self.size[1]), indexing='ij')
        pos_self = self.pos
        grid_size = self.grid_size
        gradient = np.zeros(self.size)
        
        if role == "camera":
            dist = np.sqrt((pos_self[0] - x_coords*grid_size[0])**2 + (pos_self[1] - y_coords*grid_size[1])**2) # ||q-p_i||
            x = (x_coords*grid_size[0] - pos_self[0]) # q-p_i (x direction)
            y = (y_coords*grid_size[1] - pos_self[1]) # q-p_i (y direction)
            
            ## the partial derivative of camera model w.r.t. p_i. (f_prime)
            if type == 'x':
                gradient = ((pos_self[0] - x_coords*grid_size[0])*np.exp(-((dist - self.camera_range)**2)/(2*(self.camera_variance**2)))\
                            *(self.camera_range - dist)/((self.camera_variance**2)*dist)) # ( \partial f^1(||q-p_i) ) \ ( \partial ||q-p_i|| )
                
                per_quality = (1/(1-np.cos(self.angle_of_view)))*((x*self.perspective[0] + y*self.perspective[1])/dist - np.cos(self.angle_of_view))
                gradient[per_quality < 0] = 0
                
                gradient = np.where(self.sensor_voronoi[role][event] == self.id, gradient, 0)
                
            elif type == 'y':
                gradient = ((pos_self[1] - y_coords*grid_size[1])*np.exp(-((dist - self.camera_range)**2)/(2*(self.camera_variance**2)))\
                            *(self.camera_range - dist)/((self.camera_variance**2)*dist))
                
                per_quality = (1/(1-np.cos(self.angle_of_view)))*((x*self.perspective[0] + y*self.perspective[1])/dist - np.cos(self.angle_of_view))
                gradient[per_quality < 0] = 0
                
                gradient = np.where(self.sensor_voronoi[role][event] == self.id, gradient, 0)
                
            elif type == 'perspective_x':    
                
                gradient = x/(dist)*(1-np.cos(self.angle_of_view)) # (22)
                gradient = np.where(self.sensor_voronoi[role][event] == self.id, gradient, 0)
            
            elif type == 'perspective_y':    
               
                gradient = y/(dist)*(1-np.cos(self.angle_of_view)) # (22)
                gradient = np.where(self.sensor_voronoi[role][event] == self.id, gradient, 0)
            
            gradient[np.isnan(gradient)] = 0
                
        elif role == "manipulator":
            dist = np.sqrt((pos_self[0] - x_coords*grid_size[0])**2 + (pos_self[1] - y_coords*grid_size[1])**2)
            if type == 'x':
                gradient = (2*self.approx_param*(x_coords*grid_size[0] - pos_self[0])*np.exp(-2*self.approx_param*(self.operation_range - dist)))\
                                /(((1+np.exp(-2*self.approx_param*(self.operation_range - dist)))**2)*dist)
                gradient = np.where(self.sensor_voronoi[role][event] == self.id, gradient, 0)
            elif type == 'y':
                gradient = (2*self.approx_param*(y_coords*grid_size[1] - pos_self[1])*np.exp(-2*self.approx_param*(self.operation_range - dist)))\
                                /(((1+np.exp(-2*self.approx_param*(self.operation_range - dist)))**2)*dist)
                gradient = np.where(self.sensor_voronoi[role][event] == self.id, gradient, 0)
            
            gradient[np.isnan(gradient)] = 0
            
        elif role == "smoke_detector":
            dist = (x_coords*grid_size[0] - pos_self[0])**2 + (y_coords*grid_size[1] - pos_self[1])**2
            
            if type == 'x':
                gradient = (x_coords*grid_size[0] - pos_self[0]) * np.exp(-(dist**2) / (2 * self.smoke_variance ** 2))
                gradient = np.where(self.sensor_voronoi[role][event] == self.id, gradient, 0)
                
            elif type == 'y':
                gradient = (y_coords*grid_size[1] - pos_self[1]) * np.exp(-(dist**2) / (2 * self.smoke_variance ** 2))
                gradient = np.where(self.sensor_voronoi[role][event] == self.id, gradient, 0)
            
            gradient[np.isnan(gradient)] = 0
                
        return gradient
    
    def ComputeDensityGradient(self, role, event, type):
        x_coords, y_coords = np.meshgrid(np.arange(self.size[0]), np.arange(self.size[1]), indexing='ij')
        pos_self = self.pos
        grid_size = self.grid_size
        gradient = np.zeros(self.size)
        density_gradient_x = np.zeros(self.size)
        density_gradient_y = np.zeros(self.size)
        dist = np.sqrt((pos_self[0] - x_coords*grid_size[0])**2 + (pos_self[1] - y_coords*grid_size[1])**2) # ||q-p_i||
        x = (x_coords*grid_size[0] - pos_self[0]) # q-p_i (x direction)
        y = (y_coords*grid_size[1] - pos_self[1]) # q-p_i (y direction)

        # rospy.loginfo("size of density_gradient_y: {}".format(density_gradient_x.shape))
        # rospy.loginfo("size of density_gradient_y: {}".format(density_gradient_y.shape))
        # rospy.loginfo("size of self.event_density_gradient[0]: {}".format(self.event_density_gradient[0].shape))
        # rospy.loginfo("size of self.event_density_gradient[1]: {}".format(self.event_density_gradient[1].shape))
        # rospy.loginfo("self.event_density_gradient[0]: {}".format(self.event_density_gradient[0]))
        # rospy.loginfo("self.event_density_gradient[1]: {}".format(self.event_density_gradient[1]))

        density_gradient_x += self.event_density_gradient[0]
        density_gradient_y += self.event_density_gradient[1]

        per_quality = (1/(1-np.cos(self.angle_of_view)))*((x*self.perspective[0] + y*self.perspective[1])/dist - np.cos(self.angle_of_view))
        if role == 'camera':
            if type == 'x':
                density_gradient_x[per_quality < 0] = 0 # behind the agent.
                gradient = np.where(self.sensor_voronoi[role][event] == self.id, density_gradient_x, 0) # out of the agent's Voronoi cell
            elif type == 'y':
                density_gradient_y[per_quality < 0] = 0
                gradient = np.where(self.sensor_voronoi[role][event] == self.id, density_gradient_y, 0)

        return gradient
    
    def ComputeSelfQuality(self, role, event):
        x_coords, y_coords = np.meshgrid(np.arange(self.size[0]), np.arange(self.size[1]), indexing='ij')
        pos_self = self.pos
        grid_size = self.grid_size

        self_territory = np.where(self.sensor_voronoi[role][event] == self.id, -1, self.sensor_voronoi[role][event])
        
        if role == 'manipulator':
            dist = self.operation_range - np.sqrt((pos_self[0] - x_coords*grid_size[0])**2 + (pos_self[1] - y_coords*grid_size[1])**2)
            self_quality = 1/(1+np.exp(-2*self.approx_param*(dist)))
            self_quality = np.where(self_territory > -1, 0, self_quality)
        
        elif role == 'smoke_detector':
            dist = np.sqrt((pos_self[0] - x_coords*grid_size[0])**2 + (pos_self[1] - y_coords*grid_size[1])**2)
            self_quality = np.exp(-(dist**2)/(2*self.smoke_variance**2))
            self_quality = np.where(self_territory > -1, 0, self_quality)
            
        elif role == 'camera':
            dist = np.sqrt((pos_self[0] - x_coords*grid_size[0])**2 + (pos_self[1] - y_coords*grid_size[1])**2)-self.camera_range
            self_quality = np.exp(-(dist**2)/(2*self.camera_variance**2)) # q_res
            
            dist = np.sqrt((pos_self[0] - x_coords*grid_size[0])**2 + (pos_self[1] - y_coords*grid_size[1])**2)
            x = (x_coords*grid_size[0] - pos_self[0])
            y = (y_coords*grid_size[1] - pos_self[1])
            per_quality = (1/(1-np.cos(self.angle_of_view)))*((x*self.perspective[0] + y*self.perspective[1])/dist - np.cos(self.angle_of_view)) # \hat{q}_pers
            self_quality[per_quality < 0] = 0
                    
            self_quality = np.where(self_territory > -1, 0, self_quality)
        
        return self_quality
    
    def ComputeCooperateQuality(self, coop_role, role, event = None, eval = False, coop_quality = None):
        x_coords, y_coords = np.meshgrid(np.arange(self.size[0]), np.arange(self.size[1]), indexing='ij')
        pos_self = self.pos
        grid_size = self.grid_size

        if role == 'camera':
            
            total_quality = np.zeros(self.size)
            for neighbor in self.sensor_graph[role]+[self.id]:
                valid = self.valid_sensors[role] if neighbor == self.id else (self.neighbors[neighbor]["role"][role] > 0)
                
                if valid:
                    pos = self.neighbors[neighbor]["position"] if neighbor != self.id else self.pos
                    camera_range = self.camera_range if neighbor == self.id else self.neighbors[neighbor]["camera_range"]
                    camera_variance = self.camera_variance if neighbor == self.id else self.neighbors[neighbor]["camera_variance"]
                    dist = np.sqrt((pos[0] - x_coords*grid_size[0])**2 + (pos[1] - y_coords*grid_size[1])**2) - camera_range

                    territory = np.where(self.sensor_voronoi[role][event] == neighbor, -1, self.sensor_voronoi[role][event])
                    individual_quality = np.exp(-(dist**2)/(2*camera_variance**2))
                    
                    dist = np.sqrt((pos_self[0] - x_coords*grid_size[0])**2 + (pos_self[1] - y_coords*grid_size[1])**2)
                    x = (x_coords*grid_size[0] - pos_self[0])
                    y = (y_coords*grid_size[1] - pos_self[1])
                    per_quality = (1/(1-np.cos(self.angle_of_view)))*((x*self.perspective[0] + y*self.perspective[1])/dist - np.cos(self.angle_of_view))
                    individual_quality[per_quality < 0] = 0
                    
                    individual_quality = np.where(territory > -1, 0, individual_quality)
                    
                    total_quality += individual_quality
                    
                    if eval:               
                        
                        if neighbor not in self.sensor_qualities[coop_role].keys():
                            self.sensor_qualities[coop_role][neighbor] = {}
                            self.sensor_qualities[coop_role][neighbor][event] = 0

                        elif event not in self.sensor_qualities[coop_role][neighbor].keys():
                            self.sensor_qualities[coop_role][neighbor][event] = 0
                        
                        self.sensor_qualities[coop_role][neighbor][event] += self.w_coop*np.sum(individual_quality*coop_quality*self.event_density[event])
                        
        elif role == 'manipulator':
            total_quality = np.zeros(self.size)
            for neighbor in self.sensor_graph[role]+[self.id]:
                valid = self.valid_sensors[role] if neighbor == self.id else (self.neighbors[neighbor]["role"][role] > 0)
                
                if valid:
                    pos = self.neighbors[neighbor]["position"] if neighbor != self.id else self.pos
                    operation_range = self.operation_range if neighbor == self.id else self.neighbors[neighbor]['operation_range']
                    approx_param = self.approx_param if neighbor == self.id else self.neighbors[neighbor]['approx_param']
                    dist = operation_range - np.sqrt((pos[0] - x_coords*grid_size[0])**2 + (pos[1] - y_coords*grid_size[0])**2)

                    territory = np.where(self.sensor_voronoi[role][event] == neighbor, -1, self.sensor_voronoi[role][event])
                    individual_quality = 1/(1+np.exp(-2*approx_param*(dist)))
                    individual_quality = np.where(territory > -1, 0, individual_quality)
                    
                    total_quality += individual_quality
                    
                    if eval:           

                        if neighbor not in self.sensor_qualities[coop_role].keys():
                            self.sensor_qualities[coop_role][neighbor] = {}
                            self.sensor_qualities[coop_role][neighbor][event] = 0
                            
                        elif event not in self.sensor_qualities[coop_role][neighbor].keys():
                            self.sensor_qualities[coop_role][neighbor][event] = 0

                        self.sensor_qualities[coop_role][neighbor][event] += self.w_coop*np.sum(individual_quality*coop_quality*self.event_density[event])
                        
        elif role == 'smoke_detector':
            total_quality = np.zeros(self.size, dtype=object)
            for neighbor in self.sensor_graph[role]+[self.id]:
                valid = self.valid_sensors[role] if neighbor == self.id else (self.neighbors[neighbor]["role"][role] > 0)
                
                if valid:
                    smoke_variance = self.smoke_variance if neighbor == self.id else self.neighbors[neighbor]['smoke_variance']
                    pos = self.neighbors[neighbor]["position"] if neighbor != self.id else self.pos
                    dist = np.sqrt((pos[0] - x_coords*grid_size[0])**2 + (pos[1] - y_coords*grid_size[0])**2)

                    territory = np.where(self.sensor_voronoi[role][event] == neighbor, -1, self.sensor_voronoi[role][event])
                    individual_quality = np.exp(-(dist**2)/(2*smoke_variance**2))
                    individual_quality = np.where(territory > -1, 0, individual_quality)
                    
                    total_quality += individual_quality
                    
                    if eval:          
                        
                        if neighbor not in self.sensor_qualities[coop_role].keys():
                            self.sensor_qualities[coop_role][neighbor] = {}
                            self.sensor_qualities[coop_role][neighbor][event] = 0
                            
                        elif event not in self.sensor_qualities[coop_role][neighbor].keys():
                            self.sensor_qualities[coop_role][neighbor][event] = 0
                        
                        self.sensor_qualities[coop_role][neighbor][event] += self.w_coop*np.sum(individual_quality*coop_quality*self.event_density[event])
                        
        return np.clip(total_quality, 1e-15, 10).astype('float128')
                
    def ComputeEventDensity(self, target):
        x, y = np.mgrid[0:self.map_size[0]:self.grid_size[0], 0:self.map_size[1]:self.grid_size[1]]
        xy = np.column_stack([x.flat, y.flat])
        mu = np.array(target[0]) # target mean pos
        # sigma = np.array([target[1], target[1]])
        # covariance = np.diag(sigma**2)
        covariance = np.array(target[1]).reshape((2, 2))
        z = multivariate_normal.pdf(xy, mean=mu, cov=covariance)
        event = z.reshape(x.shape)
            
        #return np.ones(self.size)

        return event

    def Norm(self, arr):

        sum = 0

        for i in range(len(arr)):
            sum += arr[i]**2

        return np.sqrt(sum)
    
    class DEBUGTOOL:
        
        def __init__(self, id):
            self.id = id
        
        def PlotSensorVoronoi(self, role, voronoi_graph):
            #print(self.id, ": ", np.unique(voronoi_graph))
            plt.imshow(voronoi_graph)
            plt.title("Sensor Voronoi of " + role)
            plt.show()
        
def KillCB(msg):
    global kill
    if msg.data == 1:
        kill = True
        
def FailureCB(msg):
    global failure
    if msg.data == 1:
        failure = True

if __name__ == "__main__":
    global kill
    global failure
    
    kill = False
    failure = False
    
    rospy.init_node('control_manager', anonymous=False, disable_signals=True)

    r = rospy.get_param("/rate", "60")
    rate = rospy.Rate(float(r))

    # Environment Parameters
    grid_size   = rospy.get_param("/grid_size", 0.1)
    grid_size   = np.array([grid_size, grid_size])
    map_width   = rospy.get_param("/map_width", 24)
    map_height  = rospy.get_param("/map_height", 24)
    map_size    = np.array([map_height, map_width])

    # General Agent's Settings
    id                      = rospy.get_param("~id", default=0)
    
    np.random.seed(id + 5)
    pos_x                   = rospy.get_param("~pos_x", default=0)
    pos_y                   = rospy.get_param("~pos_y", default=0)
    init_position           = np.array([pos_x, pos_y])#(np.random.random((1,2))*3)[0]# + 9#
    max_speed               = rospy.get_param("~max_speed", default=1)
    camera_valid            = rospy.get_param("~camera", default=1)
    manipulator_valid       = rospy.get_param("~manipulator", default=0)
    smoke_detector_valid    = rospy.get_param("~smoke_detector", default=0)
    valid_sensor            = {'camera'         : camera_valid,
                                'manipulator'    : manipulator_valid,
                                'smoke_detector' : smoke_detector_valid}
    general_info = {'id':           id,
                    'position':     init_position,
                    'valid_sensor': valid_sensor,
                    'max_speed':    max_speed}
    
    if camera_valid:
        # Camera Settings
        per_x               = rospy.get_param("~per_x", default=0)
        per_y               = rospy.get_param("~per_y", default=0)
        init_perspective    = np.array([float(per_x), float(per_y)])
        angle_of_view       = rospy.get_param("~angle_of_view")
        range_limit         = rospy.get_param("~desired_range")
        camera_variance     = rospy.get_param("~camera_variance", default=1)
        
        camera_info         = { 'perspective'    : init_perspective,
                                'angle of view'  : angle_of_view,
                                'desired range'  : range_limit,
                                'variance'       : camera_variance}
    else:
        camera_info = None
    
    if manipulator_valid:
        # Manipulator Settings
        arm_length          = rospy.get_param("~arm_length", default = 1)
        approx_param        = rospy.get_param("~approx_param", default=20)
        
        manipulator_info    = { 'arm length'     : arm_length,
                                'k'              : approx_param}
    else:
        manipulator_info = None

    if smoke_detector_valid:
        # Smoke Detector Settings
        smoke_variance      = rospy.get_param("~smoke_variance", default=1)
        smoke_detector_info = { 'variance'       : smoke_variance } 
    else:
        smoke_detector_info = None

    # Set Offboard Mode
    uav_id = id
    cmd = CMD(uav_id)

    UAV_self = PTZCamera(map_size = map_size, grid_size = grid_size, general_properties=general_info,
                        camera_properties=camera_info, smoke_detector_properties=smoke_detector_info, 
                        manipulator_properties=manipulator_info, coop = True, balance = True, strength = 10000)
    
    rospy.Subscriber("/kill", Int16, KillCB)
    rospy.Subscriber("/agent_"+str(id)+"/failure", Int16, FailureCB)
    
    frame = []
    score = []
    pos_x = []
    pos_y = []
    up_x  = []
    up_y  = []
    cnt = 0
    
    while not rospy.is_shutdown() and not kill and not failure:
        UAV_self.Update()
        # rospy.loginfo("Updating...")
            
        frame.append(cnt)
        score.append(UAV_self.total_score)
        pos_x.append(UAV_self.pos[0])
        pos_y.append(UAV_self.pos[1])
        up_x.append(UAV_self.u_p[0])
        up_y.append(UAV_self.u_p[1])
        cnt += 1
        
        plt_dict = {'frame_id'              : frame,
                    str(id)+"'s score"      : score,
                    "pos_x"                 : pos_x,
                    "pos_y"                 : pos_y,
                    "up_x"                  : up_x,
                    "up_y"                  : up_y}
            
        # df = pd.DataFrame.from_dict(plt_dict) 
        # df.to_csv (r"~/wei_research_ws/src/voronoi_cbsa/result/"+str(id)+".csv", index=False, header=True)

        save_path = "~/wei_research_ws/src/voronoi_cbsa/result/"
        isExist = os.path.exists(save_path)
        # rospy.logwarn(str(id) + ": " + str(cnt))
        if not isExist:
            os.makedirs(save_path)
            
        df = pd.DataFrame.from_dict(plt_dict) 
        df.to_csv (save_path + str(id) + ".csv", index=False, header=True)
        
        rate.sleep()
    
    # plt_dict = {'frame_id'              : frame,
    #             str(id)+"'s score"      : score,
    #             "pos_x"                 : pos_x,
    #             "pos_y"                 : pos_y}
            
    # # df = pd.DataFrame.from_dict(plt_dict) 
    # # df.to_csv (r"~/wei_research_ws/src/voronoi_cbsa/result/"+str(id)+".csv", index=False, header=True)

    # save_path = "/home/andrew/wei_research_ws/src/voronoi_cbsa/result/"
    # isExist = os.path.exists(save_path)
    # rospy.logwarn(str(id) + ": " + str(cnt))
    # if not isExist:
    #     os.makedirs(save_path)
        
    # df = pd.DataFrame.from_dict(plt_dict) 
    # df.to_csv (save_path + str(id) + ".csv", index=False, header=True)global_eventelf.cooperation