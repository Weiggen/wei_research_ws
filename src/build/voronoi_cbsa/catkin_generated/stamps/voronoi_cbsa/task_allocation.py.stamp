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
        self.agents_num = rospy.get_param('~total_agents', default=3)
        self.vehicle = "tb"
        self.agents = {}
        self.agents_initialized = False  # 新增：追蹤初始化狀態

        # camera parameters
        self.fov = rospy.get_param('~angle_of_view', default=30.0)
        self.ideal_range = rospy.get_param('~desired_range', default=3.0)
        self.camera_variance = rospy.get_param('~camera_variance', default=3.0)
        
        # Initialize targets info
        self.targets_num = 2
        self.targets = {}

        # Initialize environment variables
        real_map_width = 24
        real_map_height = 24
        self.map_size = np.array([real_map_width, real_map_height])
        self.grid_size = np.array([0.1, 0.1])
        self.size = (self.map_size / self.grid_size).astype(int)

        self.partition_map = np.full(self.size, -np.inf, dtype=float)
        self.allocation = np.zeros((self.agents_num, self.targets_num), dtype=int)
        self.RosInit()

    def RosInit(self):
        rospy.Subscriber("/target", TargetInfoArray, self.TargetCallback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.AgentCallback)
        self.pub_allocation = rospy.Publisher("/allocation", Int16MultiArray, queue_size=10)

    def TargetCallback(self, msg):
        for target in msg.targets:
            pos_x = target.position.x
            pos_y = target.position.y
            pos = np.array([pos_x, pos_y])
            cov = target.covariance
            self.targets[target.id] = [pos, cov]
        self.targets_num = len(self.targets)

    def AgentCallback(self, msg):
        agents_id = [1, 2, 3]
        previous_count = len(self.agents)
        
        for (i, name) in enumerate(msg.name):
            if name.startswith(self.vehicle + "_"):
                try:
                    agent_num = int(name.split("_")[1])
                    if agent_num in agents_id:
                        agent_pos = np.asarray([msg.pose[i].position.x, msg.pose[i].position.y])
                        rotation_matrix = R.from_quat([
                            msg.pose[i].orientation.x, 
                            msg.pose[i].orientation.y, 
                            msg.pose[i].orientation.z, 
                            msg.pose[i].orientation.w
                        ])
                        yaw = rotation_matrix.as_euler('zyx', degrees=False)[0]
                        agent_per = np.asarray([np.cos(yaw), np.sin(yaw)])
                        
                        self.agents[agent_num] = {"position": agent_pos, "perspective": agent_per}
                        
                except (ValueError, IndexError):
                    pass
        
        # 只在狀態改變時打印
        current_count = len(self.agents)
        if current_count != previous_count:
            print(f"Agents updated: {current_count}, IDs: {sorted(list(self.agents.keys()))}")
            
        # 檢查是否所有 agents 都已初始化
        if current_count == self.agents_num and not self.agents_initialized:
            self.agents_initialized = True
            print("All agents initialized successfully!")
            for agent_id in sorted(self.agents.keys()):
                print(f"Agent {agent_id}: pos={self.agents[agent_id]['position']}")

    def test(self):
        if not self.agents_initialized:
            print("Waiting for agents initialization...")
            return False
            
        print("=== Agents Status ===")
        for i in range(1, self.agents_num+1):
            if i in self.agents:
                print(f"Agent {i} position: {self.agents[i]['position']}")
            else:
                print(f"Agent {i} is missing!")
                return False
        return True

    def update_partition_map(self):
        if not self.agents_initialized:
            return False
            
        partition_map = np.full(self.size, -np.inf, dtype=float)
        for agent_id in range(1, self.agents_num+1):
            if agent_id in self.agents:
                sensor_cap = self.compute_sensor_cap(agent_id)
                partition_map = np.where(sensor_cap > partition_map, float(agent_id), partition_map)
            else:
                print(f"Agent {agent_id} not available for partition map update")
                return False
        
        self.partition_map = partition_map
        return True

    def update_ScoreMatrix(self):
        if not self.agents_initialized or len(self.targets) == 0:
            print("Cannot update score matrix: agents or targets not ready")
            return False
            
        self.score_matrix = np.zeros((self.agents_num, self.targets_num))
        
        for agent_id in range(1, self.agents_num+1):
            if agent_id not in self.agents:
                print(f"Agent {agent_id} not available for score matrix")
                return False
                
            for target_id in range(self.targets_num):
                if target_id not in self.targets:
                    print(f"Target {target_id} not available")
                    return False
                    
                local_score = 0.
                # 確保所有數組都是浮點數類型
                partition = self.partition_map.astype(float)
                partition = np.where(partition == agent_id, 1.0, 0.0)
                
                sensor_cap = self.compute_sensor_cap(agent_id=agent_id)
                event_density = self.compute_event_density(target_id=target_id)
                
                # 逐步計算，確保類型一致
                tmp = partition * sensor_cap * event_density
                local_score = np.sum(tmp)
                self.score_matrix[agent_id-1][target_id] = local_score
                
        print("Score matrix updated successfully")
        return True

    def compute_sensor_cap(self, agent_id):
        if agent_id not in self.agents:
            print(f"Agent {agent_id} not found in compute_sensor_cap")
            return np.zeros(self.size)
            
        cap = np.zeros(self.size)
        pos = self.agents[agent_id]["position"]
        per = self.agents[agent_id]["perspective"]
        
        R_range = self.ideal_range
        alpha = np.radians(self.fov)  # 轉換為弧度
        sigma = self.camera_variance
        
        def dist_vs_agent(x, y): 
            dist = np.sqrt((x - pos[0]/self.grid_size[0])**2 + (y - pos[1]/self.grid_size[1])**2)
            # 避免除零錯誤
            return np.where(dist < 1e-10, 1e-10, dist)

        q_x, q_y = np.meshgrid(np.arange(self.size[0]), np.arange(self.size[1]), indexing='ij')
        
        # Resolution quality
        distances = dist_vs_agent(q_x, q_y)
        q_res = np.exp((-(distances - R_range)**2)/(2 * sigma**2))
        
        # Perspective quality
        cos_alpha = np.cos(alpha)
        if cos_alpha >= 1.0:  # 避免除零
            cos_alpha = 0.999
            
        dot_product = ((q_x - pos[0]/self.grid_size[0])*per[0] + (q_y - pos[1]/self.grid_size[1])*per[1])
        normalized_dot = dot_product / distances
        
        q_pers = (1/(1 - cos_alpha)) * (normalized_dot - cos_alpha)
        q_pers = np.where(q_pers > 0, q_pers, 0)
        
        cap = np.where(q_pers > 0, q_res * q_pers, -1.0)  # 使用 -1.0 而不是 -np.inf

        return cap

    def compute_event_density(self, target_id):
        if target_id not in self.targets:
            return np.zeros(self.size)
            
        x, y = np.mgrid[0:self.map_size[0]:self.grid_size[0], 0:self.map_size[1]:self.grid_size[1]]
        xy = np.column_stack([x.flat, y.flat])
        pos = np.array(self.targets[target_id][0])
        cov = np.array(self.targets[target_id][1]).reshape((2, 2))
        return multivariate_normal.pdf(xy, mean=pos, cov=cov).reshape(x.shape)

    def compute_allocation_result(self):
        if not self.agents_initialized or len(self.targets) == 0:
            print("Cannot compute allocation: agents or targets not ready")
            return False
            
        model = cp_sat.CpModel()
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
                score_scaled = int(self.score_matrix[i][j] * 1000)
                objective_terms.append(score_scaled * delta[i][j])

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
            print("Allocation result:\n", allocation_result)
            self.allocation = allocation_result
            # print score of the allocation result
            score = solver.ObjectiveValue()
            print(f"Total Score (Σ_i Σ_j H[i][j] * δ_ij): {score / 1000.0:.6f}")
            return True
        else:
            print("No feasible allocation found.")
            return False

    def publish_allocation(self):
        allo = Int16MultiArray()
        allo.data = self.allocation.flatten().astype(np.int16).tolist()
        print(f"Publishing allocation: {allo.data}")
        self.pub_allocation.publish(allo)

if __name__ == "__main__":
    rospy.init_node("task_allocation", anonymous=True)
    rate = rospy.Rate(float(10))
    print("Starting task allocation node...")
    
    task_allocation = TaskAllocation()
    print("Task allocation initialized, waiting for data...")
    
    while not rospy.is_shutdown():
        if task_allocation.test():  # test() 現在會回傳布林值
            print("--- Processing cycle ---")
            
            if task_allocation.update_partition_map():
                if task_allocation.update_ScoreMatrix():
                    task_allocation.compute_allocation_result()
                    task_allocation.publish_allocation()
                else:
                    print("Score matrix update failed")
            else:
                print("Partition map update failed")
        
        rate.sleep()