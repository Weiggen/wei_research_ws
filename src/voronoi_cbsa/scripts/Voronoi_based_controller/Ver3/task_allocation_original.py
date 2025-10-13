#! /usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64, Int16MultiArray
from voronoi_cbsa.msg import TargetInfoArray
from gazebo_msgs.msg import ModelStates
from scipy.spatial.transform import Rotation as R
from scipy.stats import multivariate_normal
from ortools.sat.python import cp_model as cp_sat

class TaskAllocation():
    def __init__(self):
        # Initialize agents info
        # self.agents_num = rospy.get_param("~agents_num", 3)
        self.agents_num = 3
        self.vehicle      = "tb"
        self.agents = {}

        # camera parameters
        # self.fov = rospy.get_param("~angle_of_view", 30)
        # self.ideal_range = rospy.get_param("~desired_range", 3.0)
        # self.camera_variance = rospy.get_param("~camera_variance", 2)
        self.fov = 30
        self.ideal_range = 3
        self.camera_variance = 2
        
        # Initialize targets info
        self.targets_num = 2
        self.targets = {}

        # Initialize environment variables
        # real_map_width    = rospy.get_param("~map_width", 24)
        # real_map_height   = rospy.get_param("~map_height", 24)
        real_map_width    = 24
        real_map_height   = 24
        self.map_size     = np.array([real_map_width, real_map_height]) # 24*24
        self.grid_size = np.array([0.1, 0.1])
        self.size = (self.map_size / self.grid_size).astype(int) # 240*240 # vertex map size

        self.partition_map = np.full(self.size, -np.inf, int) # -np.inf means no territorial
        self.allocation = np.zeros((self.agents_num, self.targets_num), dtype=int)
        self.RosInit()

    def RosInit(self):
        rospy.Subscriber("/target", TargetInfoArray, self.TargetCallback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.AgentCallback)

        self.pub_allocation = rospy.Publisher("/allocation", Int16MultiArray, queue_size=10)

    def TargetCallback(self, msg):
        # targets_id = [0, 1]
        for target in msg.targets:
            pos_x = target.position.x
            pos_y = target.position.y
            pos = np.array([pos_x, pos_y])
            cov = target.covariance
            self.targets[target.id] = [pos, cov]

        self.targets_num = len(self.targets)

    # def AgentCallback(self, msg):
    #     print("Agent Callback")
    #     agents_id = [1, 2, 3]
    #     for (i, name) in enumerate(msg.name):
    #         for agent_id in agents_id:
    #             if name == self.vehicle + "_" + str(agent_id):
    #                 index = i
    #                 agent_pos = np.asarray([msg.pose[index].position.x, msg.pose[index].position.y])
    #                 rotation_matrix = R.from_quat([msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z, msg.pose[index].orientation.w])
    #                 yaw = rotation_matrix.as_euler('zyx', degrees=False)[2]
    #                 agent_per = np.asarray([np.cos(yaw), np.sin(yaw)])
    #                 self.agents[agent_id] = {"position": agent_pos, "perspective": agent_per}
    #                 print("agnet callback of {}_{}".format(self.vehicle, agent_id)) 
    #             else: break

    def AgentCallback(self, msg):        
        agents_id = [1, 2, 3]
        agents_updated = []
        
        for (i, name) in enumerate(msg.name):
            # 更高效的檢查：先檢查是否以 "tb_" 開頭
            if name.startswith(self.vehicle + "_"):
                # 提取數字部分
                try:
                    agent_num = int(name.split("_")[1])
                    if agent_num in agents_id:
                        index = i
                        agent_pos = np.asarray([msg.pose[index].position.x, msg.pose[index].position.y])
                        rotation_matrix = R.from_quat([
                            msg.pose[index].orientation.x, 
                            msg.pose[index].orientation.y, 
                            msg.pose[index].orientation.z, 
                            msg.pose[index].orientation.w
                        ])
                        yaw = rotation_matrix.as_euler('zyx', degrees=False)[0]
                        agent_per = np.asarray([np.cos(yaw), np.sin(yaw)])
                        
                        # 只在數據變化時更新和打印
                        if agent_num not in self.agents:
                            self.agents[agent_num] = {"position": agent_pos, "perspective": agent_per}
                            agents_updated.append(agent_num)
                            print(f"New agent found: {self.vehicle}_{agent_num}")
                        else:
                            # 更新位置和方向（機器人會移動）
                            self.agents[agent_num] = {"position": agent_pos, "perspective": agent_per}
                            
                except (ValueError, IndexError):
                    # 名稱格式不正確，忽略
                    pass
        
        # 只在有新 agents 或第一次時打印摘要
        if agents_updated or len(self.agents) <= 3:
            print(f"Total agents: {len(self.agents)}, IDs: {sorted(list(self.agents.keys()))}")
            if len(self.agents) == 3:
                print("All 3 agents initialized!")

    def test(self):
        for i in range(1, self.agents_num+1):
            if i in self.agents: print("agent{} position: {}".format(i, self.agents[i]["position"]))
            else: print("{} is not in self.agents list!!!".format(i))

    def update_ScoreMatrix(self):
        # Compute the score matrix based on current info
        self.score_matrix = np.zeros((self.agents_num, self.targets_num))
        '''
        @ Rows: agents [1, 2, 3]
        @ Cols: targets [0, 1]
        @ --------- Score Matrix ----------
        @   ---   | target 0  | target 1  |
        @ agent 1 |   score   |   score   | 0
        @ agent 2 |   score   |   score   | 1
        @ agent 3 |   score   |   score   | 2
        @               0           1
        '''        
        for agent_id in range(1, self.agents_num+1):
            for target_id in range(self.targets_num):

                local_score = 0.
                partition = self.partition_map
                partition = np.where(partition == agent_id, 1, 0)
                tmp = partition
                tmp *= self.compute_sensor_cap(agent_id=agent_id)
                tmp *= self.compute_event_density(target_id=target_id)
                local_score = np.sum(tmp)
                self.score_matrix[agent_id-1][target_id] = local_score
                print("Score of agent_" + str(agent_id) + " target_" + str(target_id) + " has updated.")

    def compute_allocation_result(self):
        '''
        CP-SAT
        '''
        # Initialize solver
        model = cp_sat.CpModel()
        # Define task allocation variable
        # delta[i][j] = 1 ,represent target_j is assigned to agent_i+1
        delta = []
        for i in range(self.agents_num):
            agent_vars = []
            for j in range(self.targets_num):
                var = model.NewBoolVar(f'agent_{i}_target_{j}')
                agent_vars.append(var)
            delta.append(agent_vars)

        '''
        @ Constraint 1
        @   - Each target is assigned to at least one agent.
        '''
        for j in range(self.targets_num):
            agents4target_j = [delta[i][j] for i in range(self.agents_num)]
            model.Add(sum(agents4target_j) >= 1)
        '''
        @ Constraint 2
        @   - Each agent tracking exactly one target.
        '''
        for i in range(self.agents_num):
            target4agent_i = [delta[i][j] for j in range(self.targets_num)]
            model.AddExactlyOne(target4agent_i)
        '''
        @ Objective function
        @   - Maximize : Σ_i Σ_j H[i][j] * δ_ij
        @ params
        @ H is local_score in score_matrix
        @ δ_ij allocation coefficient
        '''
        objective_terms = []
        for i in range(self.agents_num):
            for j in range(self.targets_num):
                # CP-SAT allows only integer
                score_matrix = int(self.score_matrix*1000)
                objective_terms.append(score_matrix * delta[i][j])

        total_objective = sum(objective_terms)
        model.Maximize(total_objective)
        
        solver = cp_sat.CpSolver()
        solver.parameters.max_time_in_seconds = 5
        status = solver.Solve(model)

        if status == cp_sat.OPTIMAL or status == cp_sat.FEASIBLE:
            allocation_result = np.zeros((self.agents_num, self.targets_num), dtype=int)
            for i in range(self.agents_num):
                for j in range(self.targets_num):
                    if solver.Value(delta[i][j]):
                        allocation_result[i][j] = 1
            print("Allocation result:", allocation_result)
            self.allocation = allocation_result
            return True
        else:
            print("No feasible allocation found.")
            return False

    def update_partition_map(self):
        # PTZ based partttion map
        partition_map = np.full(self.size, -np.inf, int) # -np.inf means no territorial

        for agent_id in range(1, self.agents_num+1):
            sensor_cap = self.compute_sensor_cap(agent_id)
            partition_map = np.where(sensor_cap > partition_map, agent_id, partition_map)

        self.partition_map = partition_map     
        
    def compute_sensor_cap(self, agent_id):
        cap = np.zeros(self.size)
        print("171")
        pos = self.agents[agent_id]["position"] # Error
        per = self.agents[agent_id]["perspective"]
        # Camera parameters
        R = self.ideal_range
        alpha = self.fov
        sigma = self.camera_variance
        dist_vs_agent = lambda x, y: np.sqrt((x - pos[0]/self.grid_size[0])**2 + (y - pos[1]/self.grid_size[1])**2)

        q_x, q_y = np.meshgrid(np.arange(self.size[0]), np.arange(self.size[1], indexing='ij'))
        q_res  = np.zeros(self.size)
        q_pers = np.zeros(self.size)
        # Resolution quality
        q_res = np.exp((-(dist_vs_agent(q_x, q_y) - R)**2)/(2 * sigma**2))
        # Perspective quality
        q_pers = (1/(1 - np.cos(alpha)))
        q_pers *= (((q_x - pos[0]/self.grid_size[0])*per[0] + (q_y - pos[1]/self.grid_size[1])*per[1]) 
                   / dist_vs_agent(q_x, q_y) - np.cos(alpha))
        q_pers = np.where(q_pers > 0, q_pers, 0)
        cap = np.where(q_pers > 0, q_res * q_pers, -np.inf)

        return cap
    
    def compute_event_density(self, target_id):
        x, y = np.mgrid[0:self.map_size[0]:self.grid_size[0], 0:self.map_size[1]:self.grid_size[1]]
        xy  = np.column_stack([x.flat, y.flat])
        pos = np.array(self.targets[target_id][0])
        cov = np.array(self.targets[target_id][1]).reshape((2, 2))
        return multivariate_normal.pdf(xy, mean=pos, cov=cov).reshape(x.shape)

        
if __name__ == "__main__":
    rospy.init_node("task_allocation", anonymous=True)
    rate = rospy.Rate(float(60))
    print("204")
    task_allocation = TaskAllocation()
    print("206")
    while not rospy.is_shutdown():
        print("208")
        task_allocation.test()
        print("209")
        task_allocation.update_partition_map()
        print("210")
        task_allocation.update_ScoreMatrix()
        print("212")
        task_allocation.compute_allocation_result()

        rate.sleep()