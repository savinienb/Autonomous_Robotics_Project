#!/usr/bin/env python
"""
Spyder Editor

This is a temporary script file.
"""

import roslib; roslib.load_manifest('control_turtlebot')
import rospy

from control_turtlebot.srv import *
import os,sys
import math
import numpy as np
import random as rand
import control_turtlebot


from nav_msgs.msg import Odometry




class PointGenerator(object):

    def __init__(self):
         
        #position parameters

        self.map_repr=1
        # under the form [y,x]
        self.start_po=np.zeros(2)
        self.current_position_ = np.zeros(2)
        self.goal=np.zeros(2)
        self.full_map_size=np.zeros(2)
        self.border=np.zeros(4)


        #publisher subscriber nodes/objects
       
        self.odometry_sub_ = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size = 1)
        self.direction = Odometry()

        s = rospy.Service('generate_goal', control_turtlebot.srv.GeneratePoint, self.generate_goal)

        


    def get_sub_pos(self):
    #update the repr map
         
         #recenter the map
        height=np.ceil((self.border[0]-self.border[1])/2)
        width=np.ceil((self.border[2]-self.border[3])/2)
        v_off=self.start_po[0]-height-self.border[1]
        h_off=self.start_po[1]-width-self.border[3]
            
        #find in wich part of the sub_map the robot is
        v_scale=self.full_map_size[0]/(self.map_repr/2)   
        h_scale=self.full_map_size[1]/(self.map_repr/2)
        v=np.floor((self.current_position_[0]+v_off)/v_scale)    
        h=np.floor((self.current_position_[1]+h_off)/h_scale)   
        sub_pos=[v,h]
        return sub_pos
        

    def sub_goto(self,sub_pos): 
        
        #find in wich part of the sub_map to go
        v=sub_pos[0]
        h=sub_pos[1]
        sub_goal=[0,0]
        if v==0 and h==0 :
            sub_goal=[0,1]
        elif v==0 and h==1 :         
            sub_goal=[1,1]
        elif v==1 and h==1 :
            sub_goal=[1,0]
          
        #calcul the map offset  
        height=np.floor((self.border[0]-self.border[1])/2)
        width=np.floor((self.border[2]-self.border[3])/2)
        v_off=-self.start_po[0]+height+self.border[1]
        h_off=-self.start_po[1]+width+self.border[3]
        
        #generate a point in the sub_part where the robot have to go
        
        v_pt=rand.uniform(0,self.full_map_size[0]/4)
        h_pt=rand.uniform(0,self.full_map_size[1]/4)
        v_pt+=np.ceil(self.full_map_size[0]/4)*sub_goal[0]+v_off
        h_pt+=np.ceil(self.full_map_size[0]/4)*sub_goal[1]+h_off
        
        sub_go=np.array([v_pt,h_pt])
        return sub_go
            
        
    def odomCallback(self, odometry_msg):
        #call each time the odom parameters are received
        self.current_position_[0] = odometry_msg.pose.pose.position.y
        self.current_position_[1] = odometry_msg.pose.pose.position.x
            


    def generate_goal(self,rqt):
        self.border[0]=rqt.y_max
        self.border[1]=rqt.y_min
        self.border[2]=rqt.x_max
        self.border[3]=rqt.x_min
        self.start_po[0]=rqt.origin_y
        self.start_po[1]=rqt.origin_x
        self.full_map_size[0]=rqt.y_map_size
        self.full_map_size[1]=rqt.x_map_size
  
        sub_pos=self.get_sub_pos()
        sub_goal=self.sub_goto(sub_pos)
        resp=[sub_goal[1],sub_goal[0]]
        return GeneratePointResponse(resp)
    
    

if __name__ == '__main__':
    rospy.init_node('generate_goal')
    pg = PointGenerator()
    rospy.spin()
