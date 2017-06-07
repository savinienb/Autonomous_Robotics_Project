#!/usr/bin/env python

# ROS imports
import roslib; roslib.load_manifest('control_turtlebot')
import rospy
import tf
import math

#ROS messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from control_turtlebot.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import  Clock
import random
import numpy as np
import time
from copy import deepcopy

class Controller(object):

	def __init__(self):

		#publisher subscriber nodes/objects
		self.odometry_sub_ = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size = 1)
		self.control_input_pub_ = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size = 1)
		self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.laserCallback, queue_size = 1)
		self.direction = Odometry()
		self.twist = Twist()

		self.ranges = [0]

		#position parameters
		self.origin=[0,0]
		self.current_position_ = np.zeros(2)
		self.current_orientation_ = 0.0
		self.desired_orientation_ = 0.0
		self.desired_position_ = np.zeros(2)

		#displacement parameters
		self.max_speed = [0.6,0.8]
		self.padding=1

		#boolean operator
		self.wall_left=False
		self.going_back=False
		self.origin_saved=False
		self.m_line_left=False

		self.x_map_size=50
		self.y_map_size=50
		self.x_max=0
		self.y_max=0
		self.x_min=0
		self.y_min=0

		rospy.wait_for_service('generate_goal')
		try:
			self.get_goal = rospy.ServiceProxy('generate_goal', GeneratePoint)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		self.get_new_goal()

		#timer to stop the exploration
		self.duration=rospy.Duration(20)
		self.get_back()
		return
		
	def exploration_over(self,t):

		self.going_back=True
		rospy.sleep(1)
		self.stop()
		print('planning a path to the start position')
		request = GotoWaypointRequest()
		request.goal_state_x = self.origin[0]
		request.goal_state_y = self.origin[1]
		self.go_to_start(request)
		print('robot back to the start position')


	def bug2(self):
		inc_x = self.desired_position_[0] - self.current_position_[0]
		inc_y = self.desired_position_[1] - self.current_position_[1]
		x_pos = self.current_position_[0]
		y_pos = self.current_position_[1] 

		if self.origin_saved==False:
			self.origin=deepcopy(self.current_position_)
			self.origin_saved=True

		#get parameters to know if we are on the m lines
		r,O=self.computeLine(self.current_position_, self.desired_position_)

		if self.m_line_left==False: #save the point at wich we left the m-lines
			self.leave_coord=r

		if self.going_back==False :
			try: 
				if((abs(inc_x)<0.2) and (abs(inc_y)<0.2)):#end condition
					rospy.loginfo("finished!")

				elif ((np.nanmin(self.ranges[194:347])<self.padding) and (self.m_line_left==False)): #if there is no wall and if we are on
																									#on the line
					if self.m_line_left==False:
					   self.m_line_left=True
						
					self.wall_follow()

				elif ((np.nanmin(self.ranges[194:347])>self.padding) and self.m_line_left==True and abs(r-self.leave_coord>0.1)): #is there any obastacle, are we on the m-lines, is it where we left it
				# get new point
					self.get_new_goal()

				elif ( (np.nanmin(self.ranges[194:347])>self.padding) and (self.m_line_left==False or abs(r-self.leave_coord>0.1))): #is there any obastacle, are we on the m-lines, is it where we left it
					self.go_along_line()

				else:
					if self.m_line_left==True:
						self.wall_follow()
					else:
						self.go_along_line()

			except Exception, e:

				if (self.m_line_left==True):
					self.wall_follow()
				else:
					self.go_along_line()
		

	def computeLine(self, current_position_, desired_position_):

		#return the angle difference and the distance difference
		angle_goal=math.atan2((self.desired_position_[1]-self.current_position_[1]),(self.desired_position_[0]-self.current_position_[0]))
		angle_difference=angle_goal-self.current_orientation_
		distance=math.sqrt((self.desired_position_[1]-self.current_position_[1])*(self.desired_position_[1]-self.current_position_[1])+(self.desired_position_[0]-self.current_position_[0])*(self.desired_position_[0]-self.current_position_[0]))
		if (abs(angle_difference)>3.14 ):  # find the smallest rotation

			if angle_difference<0:
				angle_difference=(6.28+angle_difference)
			else :
				angle_difference=-(6.28-angle_difference)  

		return angle_goal,angle_difference


	def go_along_line(self): #follow the m-lines
		# rospy.loginfo("on the line!")
		angular_speed=0
		linear_speed=0

		#Calculate all the parameters
		angle_goal=math.atan2((self.desired_position_[1]-self.current_position_[1]),(self.desired_position_[0]-self.current_position_[0]))
		angle_difference=angle_goal-self.current_orientation_
		distance=math.sqrt((self.desired_position_[1]-self.current_position_[1])*(self.desired_position_[1]-self.current_position_[1])+(self.desired_position_[0]-self.current_position_[0])*(self.desired_position_[0]-self.current_position_[0]))
		
		if (abs(angle_difference)>3.14 ):  # find the smallest rotation
			if angle_difference<0:
				angle_difference=(6.28+angle_difference)
			else :
				angle_difference=-(6.28-angle_difference)  

		if (abs(angle_difference)<0.1): #if the direction is good enough
			linear_speed=min(distance*5,self.max_speed[0])
			angular_speed=min(angle_difference*5,self.max_speed[1])
			self.wall=False
			if distance<self.max_speed :
					 # Waypoint reached
					 self.finished = True
		else:
					# Waypoint not reached yet
					#head the turtle to the good direction
			angular_speed=min(angle_difference*5,self.max_speed[1])
			
	  	
		self.twist.linear.x=linear_speed
		self.twist.angular.z=angular_speed
		self.control_input_pub_.publish(self.twist)


	def wall_follow(self):
		# rospy.loginfo("wall following!")
		# start here, you can't have the robot getting too close to the wall, 
		# so there must be some method in here to make is keep a cushion

		self.twist.linear.x = 0
		
		#replace nan by 9 in the sensor read
		ind = np.where(np.isnan(self.ranges))[0]
		temp=list(self.ranges)
		for i in range(0,len(ind)):
			temp[ind[i]]=9

		#cut the left side of the robot field of views in 64 parts and return in wich angle the wall is the closest
		sure=320
		quad_sz=64
		summ=np.zeros((1,quad_sz))
		for i in range(0,quad_sz):
			summ[0][i]=np.sum(temp[sure+i*320/quad_sz:sure+(i+1)*320/quad_sz])
		quad=np.argmin(summ)


		#obstacle between -[~25.7;~30] (from the robot pov)
		if quad>=55:
			self.wall_left=False
			if quad>59:
				self.twist.angular.z=self.max_speed[1]*0.5
			else:
				if summ[0][quad]>10:
					self.twist.angular.z=self.max_speed[1]*2
					self.twist.linear.x=0.4
				else:
					self.twist.angular.z=self.max_speed[1]*0.5
					
		#obstacle between -[~0;~21] (from the robot pov)
		elif quad<45:
			if self.wall_left==True:
				self.twist.linear.x=0.4
				self.twist.angular.z=self.max_speed[1]*2
			else:	
				self.twist.angular.z=-self.max_speed[1]*0.5

		else:
			self.wall_left=False
			self.twist.linear.x=0.4


		self.control_input_pub_.publish(self.twist)

	def stop(self):
		self.twist.linear.x=0
		self.twist.angular.z=-0
		self.control_input_pub_.publish(self.twist)

	def laserCallback(self, msg):
		#start below
		self.ranges = msg.ranges
		

	def odomCallback(self, odometry_msg):
		#call each time the odom parameters are received
		self.current_position_[0] = odometry_msg.pose.pose.position.x
		self.current_position_[1] = odometry_msg.pose.pose.position.y
		(r, p, y) = tf.transformations.euler_from_quaternion([odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w])
		self.current_orientation_ = y

		if self.current_position_[0]>self.x_max:
			self.x_max=self.current_position_[0]
		elif self.current_position_[0]<self.x_min:
			self.x_min=self.current_position_[0]


		if self.current_position_[1]>self.y_max:
			self.y_max=self.current_position_[1]
		elif self.current_position_[1]<self.y_min:
			self.y_min=self.current_position_[1]


		self.bug2()

		return


	def go_to_start(self, req):
		
		planner_request = FindPathToGoalRequest()
		planner_request.goal_state_x = req.goal_state_x
		planner_request.goal_state_y = req.goal_state_y
		
		planner_response = self.find_path_to_goal_serv_(planner_request)

		print('going back to the start position')
		
		for pose in planner_response.poses:
			#print pose
			control_input = Twist()
			self.desired_position_[0] = pose.x
			self.desired_position_[1] = pose.y
	 		
			loop_rate = rospy.Rate(100) # 10Hz
			orientation_approach = False
			while not rospy.is_shutdown():
				inc_x = self.desired_position_[0] - self.current_position_[0]
				inc_y = self.desired_position_[1] - self.current_position_[1]

				self.desired_orientation_ = wrapAngle(math.atan2(inc_y, inc_x))
				yaw_error = wrapAngle(self.desired_orientation_ - self.current_orientation_)
				distance_to_goal = math.sqrt(math.pow(inc_x, 2.0) + math.pow(inc_y, 2.0))
	 			
				if abs(yaw_error) > 0.04 and not orientation_approach:
					control_input.angular.x = 0.0
					control_input.angular.y = 0.0
					control_input.angular.z = yaw_error * 1.0
				
					control_input.linear.x = 0.08
					control_input.linear.y = 0.0
					control_input.linear.z = 0.0
				else:
					control_input.angular.x = 0.0
					control_input.angular.y = 0.0
					control_input.angular.z = 0.0
					
					orientation_approach = True
					liner_speed = abs(distance_to_goal) * 0.5
					
					if liner_speed < 0.1:
						control_input.linear.x = 0.1
					elif liner_speed > 0.2:
						control_input.linear.x = 0.2
					else:
						control_input.linear.x = liner_speed
					
					control_input.linear.y = 0.0
					control_input.linear.z = 0.0
	 				
				self.control_input_pub_.publish(control_input)
	 			
				rospy.logdebug("%s: current position: [%f, %f]\n", rospy.get_name(), self.current_position_[0], self.current_position_[1])
				rospy.logdebug("%s: desired position: [%f, %f]\n", rospy.get_name(), self.desired_position_[0], self.desired_position_[1])
				rospy.logdebug("%s: yaw_error: %f\n", rospy.get_name(), yaw_error)
				rospy.logdebug("%s: current orientation: %f\n", rospy.get_name(), self.current_orientation_)
				rospy.logdebug("%s: desired orientation: %f\n", rospy.get_name(), self.desired_orientation_)
				rospy.logdebug("%s: distance_to_goal: %f\n", rospy.get_name(), distance_to_goal)
				rospy.logdebug("%s: control_input.linear.x %f\n", rospy.get_name(), control_input.linear.x)
				rospy.logdebug("%s: control_input.angular.z %f\n", rospy.get_name(), control_input.angular.z)
	 			
				if distance_to_goal <= 0.2:
					break
				loop_rate.sleep()
		
		return GotoWaypointResponse()

	def  get_new_goal(self):
		# get new goal position for the robot
		temp=self.get_goal(self.x_max,self.x_min,self.y_max,self.y_min,self.x_map_size,self.y_map_size,self.origin[0],self.origin[1])
		self.desired_position_[0]=temp.goal[0]
		self.desired_position_[1]=temp.goal[1]
	
	def get_back(self):
		
		#ompl services
		self.serv_ = rospy.Service('/controller_turtlebot/goto', 
                                  GotoWaypoint, 
                                  self.go_to_start)
		
		rospy.wait_for_service('/controller_turtlebot/find_path_to_goal')
		try:
			self.find_path_to_goal_serv_ = rospy.ServiceProxy('/controller_turtlebot/find_path_to_goal', FindPathToGoal)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		#call once the service to go back
		rospy.Timer(self.duration,self.exploration_over,oneshot=True)

		


def wrapAngle(angle):

	return (angle + ( 2.0 * math.pi * math.floor( ( math.pi - angle ) / ( 2.0 * math.pi ) ) ) )
	

if __name__ == '__main__':
	rospy.init_node('control_turtlebot', log_level=rospy.INFO)
	rospy.loginfo("%s: starting turtlebot controller", rospy.get_name())

	controller = Controller()
	rospy.spin()


