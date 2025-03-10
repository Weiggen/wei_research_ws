#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from voronoi_cbsa.msg import ExchangeData, NeighborInfoArray, ExchangeDataArray, TargetInfoArray, NeighborInfo
from std_msgs.msg import Int16MultiArray, Float32MultiArray, Int16

import sys
import numpy as np

class DataCenter():
    def __init__(self, id, max_age = 3):
        self.communication_range = float(rospy.get_param("communication_range", "-1"))
        self.pos = ''
        self.id = id
        self.max_age = max_age
        self.target = {}
        self.neighbor_exchange_data = {}
        self.exchange_data = ''
        self.exchange_init = False
        self.target_init = False
        self.pos_init = False
        self.neighbor_init = False
        self.FoV_init = False
        self.global_voronoi_init = False
        self.sub_voronoi_init = False
        self.failure = False

        self.ROSInit()

    def ROSInit(self):
        self.total_agent = int(rospy.get_param("/total_agents", '3'))

        for id in range(1, self.total_agent+1):
            if id != self.id:
                rospy.Subscriber("/iris_"+str(id)+"/global/exchange_data", ExchangeData, self.NeighborCallback) # id = [1, 2, 3]
                
        rospy.Subscriber("local/exchange_data", ExchangeData, self.SelfCallback)
        rospy.Subscriber("/target", TargetInfoArray, self.TargetCallback)
        rospy.Subscriber("local/position", Point, self.PositionCallback)
        rospy.Subscriber("failure", Int16, self.FailureCB)
    
        self.pub_target2control      = rospy.Publisher("local/target", TargetInfoArray, queue_size=10)
        self.pub_neighbor2control    = rospy.Publisher("local/neighbor_info", ExchangeDataArray, queue_size=10)
        self.pub_exchange2neighbor   = rospy.Publisher("global/exchange_data", ExchangeData, queue_size=10)
        #self.pub_exchange2role       = rospy.Publisher("local/received_exchange_data", ExchangeDataArray, queue_size=10)
    
    def NeighborCallback(self, msg):
        # Retrieve the other agents' information who are within the communication range
        pos = np.array([msg.position.x, msg.position.y])
        if self.pos_init:
            self.neighbor_init = True

            if np.linalg.norm(pos - self.pos) < self.communication_range or self.communication_range == -1:
                self.neighbor_exchange_data[msg.id] = [msg, 0]  # [info, age]

            elif msg.id in self.neighbor_exchange_data:
                del self.neighbor_exchange_data[msg.id]       

    def PositionCallback(self, msg):
        self.pos_init = True
        self.pos = np.array([msg.x, msg.y])

    def SelfCallback(self, msg):
        self.exchange_init = True
        self.exchange_data = msg

    def TargetCallback(self, msg):
        self.target_init = True
        self.target = msg
    
    def CheckAge(self):
        del_list = []

        for key, item in self.neighbor_exchange_data.items():
            if item[1] < self.max_age:
                self.neighbor_exchange_data[key][1] += 1
            else:
                del_list.append(key)

        for key in del_list:
            del self.neighbor_exchange_data[key]

    def PubNeighbor2Control(self):
        msg = ExchangeDataArray()
        data = []

        for key, item in self.neighbor_exchange_data.items():
            info = item[0]
            data.append(info)

        msg.data = data
        self.pub_neighbor2control.publish(msg)

    def FailureCB(self, msg):
        if msg.data == 1:
            self.failure = True
        
    def Run(self):
        r = rospy.get_param("/rate", "60")
        rate = rospy.Rate(float(r))

        while not rospy.is_shutdown() and not self.failure:
            
            if self.target_init:
                self.pub_target2control.publish(self.target)

            if self.exchange_init:
                self.pub_exchange2neighbor.publish(self.exchange_data)

            if self.neighbor_init:
                self.CheckAge()
                self.PubNeighbor2Control()
            
            if self.global_voronoi_init:
                self.pub_global_voronoi.publish(self.global_voronoi)
                
            if self.sub_voronoi_init:
                self.pub_sub_voronoi.publish(self.sub_voronoi)
            
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('data_center', anonymous=True, disable_signals=True)
    id = rospy.get_param("~id", default=0)

    D = DataCenter(id=int(id))
        
    D.Run()
