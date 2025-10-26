#! /usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64, Int16MultiArray, Float64MultiArray
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
        self.targets_initialized = False # <<< 新增：追蹤 Target 初始化狀態

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
        self.pub_allocationScore = rospy.Publisher("/allocation_score", Float64MultiArray, queue_size=10)

    def TargetCallback(self, msg):
        target_keys_before = set(self.targets.keys())

        for idx, target in enumerate(msg.targets):
            pos_x = target.position.x
            pos_y = target.position.y
            pos = np.array([pos_x, pos_y])
            cov = target.covariance
            
            # 使用 enumerate 的 idx (0, 1, ...) 而不是 target.id
            self.targets[idx] = [pos, cov]
            # print(f"Target {idx} (original ID: {target.id}) at position {pos}") # 減少打印
        
        previous_count = self.targets_num
        self.targets_num = len(self.targets)
        target_keys_after = set(self.targets.keys())


        if previous_count != self.targets_num or target_keys_before != target_keys_after:
            print(f"Total targets: {self.targets_num}, IDs: {sorted(self.targets.keys())}")
        
        # <<< 修改：檢查是否收到了 "有效的" (non-zero) target 且尚未初始化
        
        # 檢查所有已知的 targets，看是否有任何一個的位置不是 (0, 0)
        any_non_zero_target_exists = False
        if self.targets_num > 0:
            for target_data in self.targets.values():
                # target_data[0] is the 'pos' array
                if target_data[0][0] != 0 or target_data[0][1] != 0:
                    any_non_zero_target_exists = True
                    break

        # 只有當存在非 (0,0) 的 target 且 尚未初始化時，才設置 flag
        if any_non_zero_target_exists and not self.targets_initialized:
            self.targets_initialized = True
            print("Targets initialized successfully (non-zero position detected)!")
            for target_id in sorted(self.targets.keys()):
                print(f"Target {target_id}: pos={self.targets[target_id][0]}")
        elif not any_non_zero_target_exists and self.targets_num > 0:
             # 如果我們有 targets 但全部都是 (0,0)
             if not self.targets_initialized:
                 print("Waiting for valid (non-zero) target positions... All received targets are at (0,0).")


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
        # <<< 修改：同時檢查 agents 和 targets
        if not self.agents_initialized:
            print("Waiting for agents initialization...")
            return False
            
        if not self.targets_initialized:
            print("Waiting for targets initialization...")
            return False
            
        # print("=== Agents Status ===") # 減少打印
        all_agents_present = True
        for i in range(1, self.agents_num+1):
            if i in self.agents:
                # print(f"Agent {i} position: {self.agents[i]['position']}") # 減少打印
                pass
            else:
                print(f"Agent {i} is missing!")
                all_agents_present = False
        
        return all_agents_present

    def update_partition_map(self):
        # <<< 移除：if not self.agents_initialized: ... (已由 test() 涵蓋)
        
        # 初始化
        partition_map = np.full(self.size, 0, dtype=float)
        
        # 計算每個 agent 的 sensor capability
        caps = {}
        for agent_id in range(1, self.agents_num+1):
            # <<< 移除：if agent_id in self.agents: ... (已由 test() 涵蓋)
            caps[agent_id] = self.compute_sensor_cap(agent_id)
            
        # 為每個網格點分配給 capability 最高的 agent
        for i in range(self.size[0]):
            for j in range(self.size[1]):
                max_cap = -np.inf
                best_agent = -1
                
                for agent_id, cap in caps.items():
                    if cap[i, j] > max_cap:
                        max_cap = cap[i, j]
                        best_agent = agent_id
                
                # 只有當 capability > 0 時才分配
                if max_cap > 0:
                    partition_map[i, j] = best_agent
        
        self.partition_map = partition_map
        
        # 調試輸出
        # for agent_id in range(1, self.agents_num+1): # 減少打印
        #     count = np.sum(partition_map == agent_id)
        #     print(f"Agent {agent_id} partition count: {count}")
        
        return True

    def update_ScoreMatrix(self):
        # <<< 移除：if not self.agents_initialized or len(self.targets) == 0: ... (已由 test() 涵蓋)
        
        self.score_matrix = np.zeros((self.agents_num, self.targets_num))
        
        target_ids = sorted(self.targets.keys())
        
        for agent_id in range(1, self.agents_num+1):
            # <<< 移除：if agent_id not in self.agents: ... (已由 test() 涵蓋)
            
            for matrix_idx, target_id in enumerate(target_ids):
                # 獲取該 agent 的 partition（1 或 0）
                partition = np.where(self.partition_map == agent_id, 1.0, 0.0)
                
                # 計算 sensor capability（應該都是非負值）
                sensor_cap = self.compute_sensor_cap(agent_id=agent_id)
                
                # 確保 sensor_cap 非負
                sensor_cap = np.maximum(sensor_cap, 0)
                if np.any(sensor_cap < 0):
                    rospy.logwarn(f"Agent {agent_id} has non-positive sensor capability!!!!!")
                
                # 計算 event density
                event_density = self.compute_event_density(target_id=target_id)
                
                # 計算 local score
                tmp = partition * sensor_cap * event_density
                local_score = np.sum(tmp)
                
                # 確保分數非負
                local_score = max(0, local_score)
                
                self.score_matrix[agent_id-1][matrix_idx] = local_score
                
                # print(f"Agent {agent_id}, Target {target_id}: score = {local_score:.6e}") # 減少打印
        
        print(f"Final score matrix:\n{self.score_matrix}")
        return True

    def compute_sensor_cap(self, agent_id):
        # <<< 移除：if agent_id not in self.agents: ... (已由 test() 涵蓋)
        
        pos = self.agents[agent_id]["position"]
        per = self.agents[agent_id]["perspective"]
        
        R_range = self.ideal_range
        alpha = np.radians(self.fov)
        sigma = self.camera_variance
        
        q_x, q_y = np.meshgrid(np.arange(self.size[0]), np.arange(self.size[1]), indexing='ij')
        
        # 使用實際座標而非網格索引
        x_coords = q_x * self.grid_size[0]
        y_coords = q_y * self.grid_size[1]
        
        # 計算實際距離
        dist = np.sqrt((pos[0] - x_coords)**2 + (pos[1] - y_coords)**2)
        
        # Resolution quality (基於實際距離)
        q_res = np.exp(-((dist - R_range)**2)/(2 * sigma**2))
        
        if np.any(q_res < 0):
            rospy.logwarn(f"WARNING: Agent {agent_id} has negative resolution quality values!")
            rospy.logwarn(f"  Min q_res: {np.min(q_res)}, Max q_res: {np.max(q_res)}")

        # Perspective quality
        x_diff = x_coords - pos[0]
        y_diff = y_coords - pos[1]
        
        # 避免除零
        dist_safe = np.where(dist < 1e-10, 1e-10, dist)
        
        # 正規化的點積
        dot_product = (x_diff * per[0] + y_diff * per[1]) / dist_safe
        
        cos_alpha = np.cos(alpha)
        
        q_pers = (1/(1 - cos_alpha)) * (dot_product - cos_alpha)

        q_pers = np.where(q_pers > 0, q_pers, 0)  # 確保非負

        if np.any(q_pers < 0):
            rospy.logwarn(f"WARNING: Agent {agent_id} has negative perspective quality values!")
            rospy.logwarn(f"  Min q_pers: {np.min(q_pers)}, Max q_pers: {np.max(q_pers)}")
        
        # 最終 capability 應該是非負的
        cap = q_res * q_pers
        
        # 除錯：檢查是否有負值
        if np.any(cap < 0):
            rospy.logwarn(f"WARNING: Agent {agent_id} has negative capability values!")
            rospy.logwarn(f"  Min cap: {np.min(cap)}, Max cap: {np.max(cap)}")
            cap = np.maximum(cap, 0)  # 強制非負
        
        return cap
    
    # def compute_sensor_cap(self, agent_id):
    #     if agent_id not in self.agents:
    #         print(f"Agent {agent_id} not found in compute_sensor_cap")
    #         return np.zeros(self.size)
            
    #     cap = np.zeros(self.size)
    #     pos = self.agents[agent_id]["position"]
    #     per = self.agents[agent_id]["perspective"]
        
    #     R_range = self.ideal_range
    #     alpha = np.radians(self.fov)  # 轉換為弧度
    #     sigma = self.camera_variance
        
    #     def dist_vs_agent(x, y): 
    #         dist = np.sqrt((x - pos[0]/self.grid_size[0])**2 + (y - pos[1]/self.grid_size[1])**2)
    #         # 避免除零錯誤
    #         return np.where(dist < 1e-10, 1e-10, dist)

    #     q_x, q_y = np.meshgrid(np.arange(self.size[0]), np.arange(self.size[1]), indexing='ij')
        
    #     # Resolution quality
    #     distances = dist_vs_agent(q_x, q_y)
    #     q_res = np.exp((-(distances - R_range)**2)/(2 * sigma**2))
        
    #     # Perspective quality
    #     cos_alpha = np.cos(alpha)
    #     if cos_alpha >= 1.0:  # 避免除零
    #         cos_alpha = 0.999
            
    #     dot_product = ((q_x - pos[0]/self.grid_size[0])*per[0] + (q_y - pos[1]/self.grid_size[1])*per[1])
    #     normalized_dot = dot_product / distances
        
    #     q_pers = (1/(1 - cos_alpha)) * (normalized_dot - cos_alpha)
    #     q_pers = np.where(q_pers > 0, q_pers, 0)
        
    #     cap = np.where(q_pers > 0, q_res * q_pers, -1.0)  # 使用 -1.0 而不是 -np.inf

    #     return cap

    def compute_event_density(self, target_id):
        if target_id not in self.targets:
            return np.zeros(self.size)
            
        x, y = np.mgrid[0:self.map_size[0]:self.grid_size[0], 0:self.map_size[1]:self.grid_size[1]]
        xy = np.column_stack([x.flat, y.flat])
        pos = np.array(self.targets[target_id][0])
        cov = np.array(self.targets[target_id][1]).reshape((2, 2))
        return multivariate_normal.pdf(xy, mean=pos, cov=cov).reshape(x.shape)

    def compute_allocation_result(self):
        # <<< 移除：if not self.agents_initialized or len(self.targets) == 0: ... (已由 test() 涵蓋)
            
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

    def publish_allocation_score(self):
        score_msg = Float64MultiArray()
        score_msg.data = self.score_matrix.flatten().tolist()
        print(f"Publishing allocation score: {score_msg.data}")
        self.pub_allocationScore.publish(score_msg)

if __name__ == "__main__":
    rospy.init_node("task_allocation", anonymous=True)
    rate = rospy.Rate(float(10))
    print("Starting task allocation node...")
    
    task_allocation = TaskAllocation()
    print("Task allocation initialized, waiting for data...")
    
    while not rospy.is_shutdown():
        if task_allocation.test():  # test() 現在會回傳布林值 (且檢查 agent 和 target)
            print("--- Processing cycle ---")
            
            if task_allocation.update_partition_map():
                if task_allocation.update_ScoreMatrix():
                    task_allocation.compute_allocation_result()
                    task_allocation.publish_allocation()
                    task_allocation.publish_allocation_score()
                else:
                    print("Score matrix update failed")
            else:
                print("Partition map update failed")
        
        rate.sleep()