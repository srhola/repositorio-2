#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64
import message_filters
#The code came from: https://stackoverflow.com/questions/35145555/python-real-time-plotting-ros-data
#documentation about "self": https://www.w3schools.com/python/gloss_python_self.asp 

class Visualiser:
    def __init__(self):
        self.fig, (self.ax0,self.ax1,self.ax2) = plt.subplots(3)
        self.ln0, = self.ax0.plot([], [], 'r.')
        self.ln1, = self.ax1.plot([], [], 'r.')
        self.ln2, = self.ax2.plot([], [], 'r.')
        self.x_data, self.y_data0, self.y_data1, self.y_data2 = [] , [] , [] , []


    def plot_init(self):
        #viento
        self.ax0.set_xlim(0, 1000)
        self.ax0.set_ylim(-4, 4)
        self.ax0.set_title('roll x')
        self.ax0.set_xlabel("Tiempo [s]")
        self.ax0.set_ylabel("angulo [rad]")  
        self.ax1.set_xlim(0, 1000)
        self.ax1.set_ylim(-3.14, 3.14)
        self.ax1.set_title('pitch')
        self.ax1.set_xlabel("Tiempo [s]")
        self.ax1.set_ylabel("angulo [rad]")  
        self.ax2.set_xlim(0, 1000)
        self.ax2.set_ylim(-3.14, 3.14)
        self.ax2.set_title('yaw z')
        self.ax2.set_xlabel("Tiempo [s]")
        self.ax2.set_ylabel("angulo [rad]")    
        '''
        #posicion
        self.ax0.set_xlim(0, 1000)
        self.ax0.set_ylim(-200, 200)
        self.ax0.set_title('Direccion x')
        self.ax1.set_xlim(0, 1000)
        self.ax1.set_ylim(-200, 200)
        self.ax1.set_title('Direccion y')
        self.ax2.set_xlim(0, 1000)
        self.ax2.set_ylim(-10, 200)
        self.ax2.set_title('Direccion z')
        '''
        #listo        
        return self.ln0
    
    def getPOS(self, pose):
        quaternion = (pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        POS_x = quaternion[0] 
        POS_y = quaternion[1]
        POS_z = quaternion[2]
        #listo           
        return (POS_x,POS_y,POS_z)
    def getAttitude(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        POS_x = euler[0] 
        POS_y = euler[1]
        POS_z = euler[2]
        #listo           
        return (POS_x,POS_y,POS_z)
    '''
    def getFLOAT64(self, msg):
        valor = msg.data
        #listo           
        return valor

    '''
    def odom_callback(self, msg, phoenic):
        #posicion= self.getPOS(msg.pose.pose)
        angulo= self.getAttitude(msg.pose.pose)
        self.y_data0.append(angulo[0])
        self.y_data1.append(angulo[1])
        self.y_data2.append(angulo[2])
        print('roll',angulo[0],'pitch',angulo[1],'yaw',angulo[2])
        #self.y_data0.append(posicion[0])
        #self.y_data1.append(posicion[1])
        #self.y_data2.append(posicion[2])
        x_index = len(self.x_data)/1
        self.x_data.append(x_index+1/1000)
        #listo
    def float_callback(self, msg):
        #posicion= self.getFLOAT64(msg.data)
        posicion=msg.data
        print(posicion)
        zero=0
        zerotwo=0
        self.y_data0.append(posicion)
        self.y_data1.append(zero)
        self.y_data2.append(zerotwo)
        x_index = len(self.x_data)/1
        self.x_data.append(x_index+1/1000)
        #listo    
    def update_plot(self, frame):
        self.ln0.set_data(self.x_data, self.y_data0)
        self.ln1.set_data(self.x_data, self.y_data1)
        self.ln2.set_data(self.x_data, self.y_data2)
        return self.ln0


rospy.init_node('listener1')
vis = Visualiser()
#sub = rospy.Subscriber('chatter', Float64, vis.float_callback)
#sub = rospy.Subscriber('wind_state', Odometry, vis.odom_callback)
#odom1_sub = rospy.Subscriber('drone1/odom', Odometry, vis.odom_callback)
odom1_sub = message_filters.Subscriber('/drone1/odom', Odometry)
odom2_sub = message_filters.Subscriber('/drone2/odom', Odometry)
#ts = message_filters.TimeSynchronizer([odom1_sub], 10)
ts = message_filters.TimeSynchronizer([odom1_sub,odom2_sub], 10)
ts.registerCallback(vis.odom_callback)


ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 