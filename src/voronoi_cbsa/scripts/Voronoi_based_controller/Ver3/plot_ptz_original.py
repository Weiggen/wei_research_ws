#! /usr/bin/env python3
import rospy
import pygame
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import Pose
from voronoi_cbsa.msg import TargetInfoArray, ValidSensors, WeightArray
from scipy.stats import multivariate_normal

class Visualization():
    def PoseCB(self, id):
        def callback(msg):
            # agent_id is [1, 2, 3]
            self.agent_pos[id] = np.asarray([msg.position.x, msg.position.y])
            self.agent_per[id] = np.asarray([msg.orientation.x, msg.orientation.y])
        return callback

    def TargetCallback(self, msg):
        self.target_received = True
        self.targets = {}
        # target.id is [0, 1]
        for target in msg.targets:
            pos_x = target.position.x
            pos_y = target.position.y
            pos = np.array([pos_x, pos_y])
            cov = target.covariance
            weight = target.weight
            vel_x = target.velocity.linear.x
            vel_y = target.velocity.linear.y
            vel = np.array([vel_x, vel_y])
            requirements = [target.required_sensor[i] for i in range(len(target.required_sensor))]
            self.targets[target.id] = [pos, cov, weight, vel, requirements]

    def __init__(self):
        # Initialization robots parameters
        self.total_agents = rospy.get_param("~total_agents", 3)
        self.vehicle      = "tb"
        self.agent_pos = {}
        self.agent_per = {}
        self.target_received = False
        self.targets = {}
        
        # Initialize environment variables
        real_map_width    = rospy.get_param("~map_width", 24)
        real_map_height   = rospy.get_param("~map_height", 24)
        self.map_size     = np.array([real_map_width, real_map_height]) # 24*24

        grid_size = rospy.get_param("~grid_size", 0.1)
        self.grid_size = np.array([grid_size, grid_size])
        self.vertex_map_size = (self.map_size / self.grid_size).astype(int) # 240*240

        # Initialize camera parameters
        self.fov = rospy.get_param("~angle_of_view", 30)
        self.ideal_range = rospy.get_param("~desired_range", 3.0)
        self.camera_variance = rospy.get_param("~camera_variance", 2)

        # For updating robots' info
        # - Subscribers for agents
        for i in range(1, self.total_agents):
            rospy.Subscriber("/"+self.vehicle+"_"+str(i+1)+"/visualize/pose", Pose, self.PoseCB(i)) 
            # including 2D position & perspective(2D vector)

        # - Subscribers for targets
        rospy.Subscriber("/target", TargetInfoArray, self.TargetCallback)

        # pygame setup
        self.block_size = 4
        self.window_size = self.vertex_map_size * self.block_size
        self.screen = pygame.display.set_mode(self.window_size)
        # Initialize pygame evironment
        self.screen.fill((0, 0, 0)) # Fill the screen with black
        # Parameters for drawing
        self.color_pool = [(175, 100, 100), (100, 150, 100), (100, 100, 150), 
                      (255, 0  , 0  ), (0  , 255, 255), (0  , 0  , 255), 
                      (178, 102, 255), (255, 0  , 255), (13 , 125, 143)]

    def Update(self):

        def update_agent_info(self, id):
            pos = self.agent_pos[id]
            per = self.agent_per[id]
            # Update agent positions and orientations
            pygame.draw.circle(surface=self.screen,
                                color=self.color_pool[id % len(self.color_pool)],
                                center=(int(pos[0] * self.block_size), 
                                        int(pos[1] * self.block_size)),
                                radius=2)
            # Draw the agent's orientation as a line
            orientation_end = (pos[0] + per[0]*self.ideal_range, 
                                pos[1] + per[1]*self.ideal_range)
            pygame.draw.line(surface=self.screen,
                                color=self.color_pool[id % len(self.color_pool)],
                                start_pos=(int(pos[0] * self.block_size), 
                                        int(pos[1] * self.block_size)),
                                end_pos=(int(orientation_end[0] * self.block_size), 
                                        int(orientation_end[1] * self.block_size)),
                                width=1)

            # Font setup for displaying agent ID
            bias = 30
            agent_name = self.vehicle + "_" + str(id)
            font = pygame.font.Font(None, 24)
            text_center = (int(pos[0] * self.block_size), 
                            int(pos[1] * self.block_size) - bias)
            text = font.render(agent_name, True, (0, 0, 0))
            text_rect = text.get_rect(center=text_center)
            self.screen.blit(text, text_rect) 

        def update_partition(self, id):
            partition_map = np.full(self.vertex_map_size, -1) # 240*240 full of -1
            for id in range(1, self.total_agents):
                pos = self.agent_pos[id]
                per = self.agent_per[id]
                alpha = self.fov
                R = self.ideal_range
                sigma = self.camera_variance

                q_res = np.zeros(self.vertex_map_size)
                q_per = np.zeros(self.vertex_map_size)
                for q_x, x in enumerate(range(0, self.vertex_map_size[0], self.block_size)):
                    for q_y, y in enumerate(range(0, self.vertex_map_size[1], self.block_size)):
                        dist = np.sqrt((pos[0] - x)**2 + (pos[1] - y)**2)
                        q_res = np.exp((-(dist - R)**2)/(2*sigma**2))
                        q_per = (1/(1 - np.cos(alpha)))
                        q_per *= (((x-pos[0])*per[0] + (y-pos[1])*per[1])/dist - np.cos(alpha))
                        camera_score = q_res * q_per if q_per > 0 else 0
                        partition_map[q_y, q_x] = id if camera_score > partition_map[q_y, q_x] else partition_map[q_y, q_x]

                # Now we have the partition_map with the corresponding id in the territory
            return partition_map
        
        def update_target_info(self, target_id):
            # Update target positions and covariances
            pos = self.targets[target_id][0]
            cov = self.targets[target_id][1]

            def event_density(self, pos, cov):
                # Create a grid for the event density
                x = np.linspace(0, self.map_size[0], self.vertex_map_size[0])
                y = np.linspace(0, self.map_size[1], self.vertex_map_size[1])
                X, Y = np.meshgrid(x, y)
                pos_grid = np.dstack((X, Y))
                # Calculate the probability density function
                rv = multivariate_normal(mean=pos, cov=cov)
                return rv.pdf(pos_grid)
            
            # Plot the event density w/ whiteness
            # - Position w/ a dot & ID
            pygame.draw.circle(surface=self.screen,
                                color=(255, 255, 255),
                                center=(int(pos[0] * self.block_size), 
                                        int(pos[1] * self.block_size)),
                                radius=3)
            font = pygame.font.Font(None, 24)
            text_center = (int(pos[0] * self.block_size), 
                            int(pos[1] * self.block_size) - 15)
            text = font.render("Target_"+str(target_id), True, (255, 255, 255))
            text_rect = text.get_rect(center=text_center)
            self.screen.blit(text, text_rect)
            # - Covariance w/ whiteness
            #   - Guarantee the pdf is normalized to [0, 1]
            density = event_density(pos, cov)
            normalized_density = (density - np.min(density)) / (np.max(density) - np.min(density))
            density4plot = (normalized_density * 255).astype(np.uint8)
            #   - Paint the whiteness on the current screen
            for agent_id in range(1, self.total_agents):
                partition_map = update_partition(self, id)

            for agent_id in range(1, self.total_agents):
                for x_map, x in enumerate(range(0, self.window_size[0], self.block_size)):
                    # [0:4:480] for 240*240
                    for y_map, y in enumerate(range(0, self.window_size[1], self.block_size)):
                        territorial_id = partition_map[y_map, x_map]
                        if territorial_id != -1:
                            color = self.color_pool[territorial_id % len(self.color_pool)]
                            brightness = density4plot[y_map, x_map]
                            shaded_color = (min((color[0] + brightness)/2, 255),
                                            min((color[1] + brightness)/2, 255),
                                            min((color[2] + brightness)/2, 255))
                            pygame.draw.rect(surface=self.screen,
                                                color=shaded_color,
                                                rect=(x, y, self.block_size, self.block_size))
        
        for agent_id in range(1, self.total_agents):
            update_agent_info(self, agent_id)

        if self.target_received:
            for target_id in self.targets.keys():
                update_target_info(self, target_id)
        
        pygame.display.flip()  # Update the full display Surface to the screen
                

if __name__ == "__main__":
    rospy.init_node("visualization_node")
    pygame.init()
    
    visual = Visualization()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        visual.Update()
        rate.sleep()

    pygame.quit()  # Quit pygame when done