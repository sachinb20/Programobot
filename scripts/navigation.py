#!/usr/bin/env python
import rospy
import numpy as np
import math
from sensor_msgs.msg import NavSatFix 
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32                                                                    #   XY PLANE
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion

byke_coordi=np.array([])

class navigate:
		
	def direct_handle_towards_dstn(self):#called if no obstacle is detected this will keep handle neary towards destination
		if (math.sqrt(sum(i**2 for i in (self.unit_vel_vtr - self.unit_dstn_byke_vtr)))<.5):      
			self.handle_msg.data=0                                                      
			self.omega(self.handle_msg)					
			rospy.loginfo("------on the line--------")        #if handle nearly toward destination
			pass
		else:
			rospy.loginfo("--------diverted---------")     #if handle is diverted from destination
			dstn_theta=math.degrees(math.atan(self.unit_dstn_byke_vtr[0]/self.unit_dstn_byke_vtr[1]))   #finding angle between unitvelocity vector and  destinton vectot
			vel_theta=math.degrees(math.atan(self.unit_vel_vtr[0]/self.unit_vel_vtr[1]))
			self.diff_angle=dstn_theta - vel_theta 
			rospy.loginfo(self.diff_angle)
			if self.diff_angle > 0:                          #   if diff is positive then handle move towardsright
				if not (self.handle_JS > 0.75):          #condition for no to rotate handle more than .75 radian
					rospy.loginfo("-----positive-----")
					self.handle_msg.data=1*self.handle_speed
				elif (self.handle_JS > 0.75):
					self.handle_msg.data=0	
				
			elif self.diff_angle < 0:                        #   if diff is negative then handle move towards left
				if not (self.handle_JS < -0.75):         #condition for no to rotate handle more than .75 radian
					rospy.loginfo("-----negative-----")
					self.handle_msg.data=-1*self.handle_speed
				elif(self.handle_JS < -0.75):            #condition for no to rotate handle more than .75 radian
					self.handle_msg.data=0	
			rospy.loginfo(self.handle_msg)	
			self.omega.publish(self.handle_msg)
	
	def get_unit_vtr(self,v):      #will return unit vector
		vtr= (v.astype(float)/ np.sqrt(np.sum(v**2)))  
		return vtr
		
	def lla_to_xyz(self,lla):        #converting longitude latitude to XYZ coordinates
		earthRadius = 6378.137
		x = earthRadius * math.cos(lla[0])*math.cos(lla[1])
		y = earthRadius * math.cos(lla[0])*math.sin(lla[1])
		z = earthRadius * math.sin(lla[0])
		return np.array([x,y,z])

	def get_location(self,lla):     #call back for /sbb/gps
		location=[lla.latitude,lla.longitude]
		global byke_coordi 
		byke_coordi= self.lla_to_xyz(location)
		self.unit_dstn_byke_vtr=self.get_unit_vtr(self.dstn_coordi-byke_coordi)

	def get_vel_vector3(self,data):#call back for sbb/gps/vel
		self.unit_vel_vtr=self.get_unit_vtr(np.array([data.vector.x,data.vector.y,data.vector.z]))  #storing unit vel vector of cycle by calling get_unit_vtr function
				
	
	def get_handle_joint_state(self,data):   #call back for /handle/joint_state 
		self.handle_JS=data.position[0]   
		
	def get_lidar_info(self,data):    # call back for /sbb/distance_sensor/front
		self.ranges=data.ranges
		index=1
		self.obst_detected=0
		rospy.loginfo("lidar_rngs:{}".format(self.ranges))                                                                        #ranges
		for value in self.ranges:            #loop if obstacle detected in specified range
			if value<self.lidar_range : 
				self.speed_msg.data=5.0   
				self.pub.publish(self.speed_msg)
				if (((((index==9 or index==10) or index==11) or index==12) or index==13) or index ==14) :   #right side region
					self.obst_detected=1
					rospy.loginfo("obstacle detect at right ")
					if (self.handle_JS > -0.75):                       #condition for no to rotate handle more than .75 radian
						self.handle_msg.data=-1*self.handle_speed  #if obstacle detected at rght side then it will rotate on left side 
						self.omega.publish(self.handle_msg)
					else:
						self.handle_msg.data=0		

				if  (((((index==3 or index==2) or index==4) or index==5) or index==6) or index ==7):     #left side region
					rospy.loginfo(value)
					self.obst_detected=1					
					rospy.loginfo("obstacle detect at left")
					if  (self.handle_JS < 0.75):                   #condition for no to rotate handle more than .75 radian
						self.handle_msg.data=1*self.handle_speed #if obstacle detected at left side then it will rotate on right side 
						self.omega.publish(self.handle_msg)
					else:
						self.handle_msg.data=0	
				if index==8 :
					rospy.loginfo(value)
					self.obst_detected=1					
					rospy.loginfo("obstacle detected at front")
					self.handle_msg.data=-1*self.handle_speed
					self.omega.publish(self.handle_msg)
			index=index+1
			if index==15 and self.obst_detected==0:             #if path is clear than it will call diect_handle_towards dstn function
				self.speed_msg.data=self.speed
				self.pub.publish(self.speed_msg)
				rospy.loginfo("no obstacle detected")
				self.direct_handle_towards_dstn()

	def __init__(self,speed,handle_speed,lidar_range):
		self.handle_speed=handle_speed
		self.lidar_range=lidar_range
		self.speed_msg=Float32()                #publishing cycle speed msg
		self.handle_msg=Float32()               #publishing handle speed msg
		self.speed=speed              
		self.obst_detected=0                    #boolean for obstacle msg
		global byke_coordi 
		rospy.init_node('to_final_destination',anonymous=True)
		self.pub=rospy.Publisher('/drive_wheel/command',Float32,queue_size=10)
		self.omega=rospy.Publisher('/handle/command',Float32,queue_size=10)
		self.wheel_comd=rospy.Publisher('/flywheel/command',Float32,queue_size=10)
		self.dstn_coordi=self.lla_to_xyz([10.0001415852, 10.0003820894])
		r=rospy.Rate(50)
		while not rospy.is_shutdown():
			try:    
				self.speed_msg.data=self.speed
				self.pub.publish(self.speed_msg)
				rospy.Subscriber('/sbb/distance_sensor/front',LaserScan,self.get_lidar_info) 
				rospy.Subscriber('/sbb/gps',NavSatFix,self.get_location)
				rospy.Subscriber('/handle/joint_state',JointState,self.get_handle_joint_state)
				rospy.Subscriber('/sbb/gps/vel',Vector3Stamped,self.get_vel_vector3)

		 
				r.sleep()
			except rospy.exceptions.ROSTimeMovedBackwardsException: 
				pass
			
	
if __name__=='__main__':
	lidar_range=1.7       #selected lidar range till obstacle got detected
	speed=10              #constant speed of cycle
	handle_speed=.40      #handle rotating speed
	navigate(speed,handle_speed,lidar_range)  # class call