#!/usr/bin/env python
import matplotlib.pyplot as plt
import statistics as st
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64
#The code came from: https://stackoverflow.com/questions/35145555/python-real-time-plotting-ros-data
#documentation about "self": https://www.w3schools.com/python/gloss_python_self.asp 

class Visualiser:
    def __init__(self):
        self.fig, (self.ax0) = plt.subplots(1)
        self.ln0, = self.ax0.plot([], [], 'r.')
        self.x_data, self.y_data0 = [] , [] 


    def plot_init(self):
        #viento
        '''
        self.ax0.set_xlim(0, 60)
        self.ax0.set_ylim(10, 30)
        self.ax0.set_title('Distancia estimada con camaras')
        self.ax0.set_xlabel("Tiempo [s]")
        self.ax0.set_ylabel("Distancia estimada [m]")        
        '''
        self.ax0.set_xlim(0, 60)
        self.ax0.set_ylim(-1, 1)
        self.ax0.set_title('Distancia estimada con lidar')
        self.ax0.set_xlabel("Tiempo [s]")
        self.ax0.set_ylabel("Error lidar [m]")     
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

    def odom_callback(self, msg):
        posicion= self.getPOS(msg.pose.pose)
        self.y_data0.append(posicion[0])
        self.y_data1.append(posicion[1])
        self.y_data2.append(posicion[2])
        x_index = len(self.x_data)/1

        self.x_data.append(x_index+1/1000)
        #listo
    def float_callback(self, msg):
        #posicion= self.getFLOAT64(msg.data)
        original=99.9254
        posicion=msg.data-original
        #print(posicion)
        self.y_data0.append(posicion)
        if len(self.x_data)>2:
            estandar=st.stdev(self.y_data0)
            media=st.mean(self.y_data0)
            print("stddev muestral:",estandar,"max:", max(self.y_data0),"min:",min(self.y_data0),"mean:", media)
        #x_index = len(self.x_data)/10
        x_index = len(self.x_data)/2
        print(x_index)
        #self.x_data.append(x_index+1/1000)
        self.x_data.append(x_index+0.5)
        #listo    
    def update_plot(self, frame):
        self.ln0.set_data(self.x_data, self.y_data0)
        return self.ln0


rospy.init_node('listener1')
vis = Visualiser()
#sub = rospy.Subscriber('chatter', Float64, vis.float_callback)
sub = rospy.Subscriber('/distancia_lidar', Float64, vis.float_callback)

#sub = rospy.Subscriber('wind_state', Odometry, vis.odom_callback)
#sub = rospy.Subscriber('drone1/realilinkpose', Odometry, vis.odom_callback)
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 