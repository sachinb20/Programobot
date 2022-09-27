#!/usr/bin/env python
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Quaternion,Twist
import rospy
import tf

class sbb():

    def __init__(self):   

        rospy.init_node('controller',anonymous=True)

        self.sbb_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
        self.sbb_orientation_euler = [0.0, 0.0, 0.0]


        self.sample_rate = 50.0



        self.curr_angle = 0.0   # Rotated angle of sbb wrt z axis
        self.prev_angle = 0.0
        self.w1=0.0  #angular velocity of the handle
        self.I=0    #Integral controller

        self.data_cmd = Float32()  #command data
        self.data_pub = rospy.Publisher('/flywheel/command', Float32, queue_size=10)
        rospy.Subscriber('/sbb/imu', Imu, self.imu_callback)

    
    def imu_callback(self, msg): #callback function for /sbb/imu

        self.sbb_orientation_quaternion = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]

    def pid(self):  #PID controller
            
        (self.sbb_orientation_euler[1], self.sbb_orientation_euler[0], self.sbb_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.sbb_orientation_quaternion[0], self.sbb_orientation_quaternion[1], self.sbb_orientation_quaternion[2], self.sbb_orientation_quaternion[3]])
        self.curr_angle = self.sbb_orientation_euler[1]
        rospy.loginfo("eulers: " + str(self.sbb_orientation_euler))
        kp1=7.3  #PID controller 1 constants
        kd1=0.020
        ki1=0.020
        kp2=137  #PID controller 2 constants
        kd2=0.25
        dt=0.02
        ki2=0.020 
        self.I=self.I + self.curr_angle*dt

        self.w1= self.w1- (kp1*self.curr_angle)-(kd1*(self.curr_angle-self.prev_angle)/dt)-(ki1*self.I)    
        self.w2= - (kp2*self.curr_angle)-(kd2*(self.curr_angle-self.prev_angle)/dt)-(ki2*self.I)     
        self.w=self.w1+self.w2 

        self.data_cmd.data= self.w
        rospy.loginfo("flywheel w: "+str(self.w))
        self.data_pub.publish(self.data_cmd)

        self.prev_angle=self.curr_angle
               
if __name__ == '__main__':
    sbb = sbb()
    r = rospy.Rate(sbb.sample_rate)
    try:  
        while not rospy.is_shutdown():    
            sbb.pid()
            r.sleep()
    except ValueError as e:
            rospy.loginfo(e)