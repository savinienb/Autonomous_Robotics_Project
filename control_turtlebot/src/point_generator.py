#!/usr/bin/env python
"""
Spyder Editor

This is a temporary script file.
"""
import os,sys
import math
import numpy as np
import random as rand

global full_map
global repr_map

def get_sub_pos(coord,full_map_size,border,start_po,map_repr):
#update the repr map
    if full_map[coord[0]][coord[1]]==1.0:
     
        #recenter the map
        height=np.ceil((border[0]-border[1])/2)
        width=np.ceil((border[2]-border[3])/2)
        v_off=start_po[0]-height-min_y
        h_off=start_po[1]-width-min_x
        
        #find in wich part of the sub_map the robot is
        v_scale=full_map_size[0]/(map_repr/2)   
        h_scale=full_map_size[1]/(map_repr/2)
        v=np.floor((coord[0]+v_off)/v_scale)    
        h=np.floor((coord[1]+h_off)/h_scale)   
        sub_pos=[int(v),int(h)]
        return sub_pos
    

def sub_goto(sub_pos,border,full_map_size): 
    
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
    height=np.floor((border[0]-border[1])/2)
    width=np.floor((border[2]-border[3])/2)
    v_off=-start_po[0]+height+min_y
    h_off=-start_po[1]+width+min_x
    
    #generate a point in the sub_part where the robot have to go
    v_pt=rand.uniform(0,np.floor(full_map_size[0]/4))
    h_pt=rand.uniform(0,np.floor(full_map_size[1]/4))
    v_pt+=np.ceil(full_map_size[0]/4)*sub_goal[0]+v_off
    h_pt+=np.ceil(full_map_size[0]/4)*sub_goal[1]+h_off
    
    sub_go=np.array([v_pt,h_pt])
    return sub_go
        
    





map_size=np.array([4,4])
map_resolution=1

if map_resolution<1:
    if (math.log(int(1/map_resolution),int(4))).is_integer()==0:
        map_resolution=1/(np.power(4,math.ceil(math.log(int(1/map_resolution),int(4)))))
full_map_size=map_size*map_resolution*4
map_repr=4

#sanity check
if map_repr%4 != 0:
    map_repr+=4
    map_repr-=map_repr%4
    print('warning reprsentation')
if map_size[0]%map_repr!=0:
    full_map_size[0]+=map_repr    
    full_map_size[0]-=full_map_size[0]%map_repr
    print('warning 1')
if map_size[1]%map_repr!=0:
    full_map_size[1]+=map_repr
    full_map_size[1]-=full_map_size[1]%map_repr
    print('warning 2')
                       
                       
                      
 #prepare the map
full_map=np.zeros(full_map_size) 
repr_map=np.zeros([map_repr/2,map_repr/2])          
         
start_po=np.asarray((full_map_size/2), dtype=int)
coord=np.array([11,3])
coord=np.floor(coord*map_resolution)
#sanity check
coord[coord==full_map.shape[1]]=full_map.shape[1]-1

#add the point
full_map[coord[0]][coord[1]]+=1.0


max_x=8
min_x=0
max_y=15
min_y=0

border=[max_y,min_y,max_x,min_x]

sub_pos=get_sub_pos(coord,full_map_size,border,start_po,map_repr)
#recenter the map
sub_go=sub_goto(sub_pos,border,full_map_size)

sub_go=sub_go/map_resolution



