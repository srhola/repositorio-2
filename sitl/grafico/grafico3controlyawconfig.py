#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
import message_filters
import socket
import pickle
import threading 
import math
import statistics as st

from std_msgs.msg import String

#https://dronekit-python.readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
#####################dependencies###########################
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, LocationLocal,  LocationGlobal
import time

import exceptions
import math
import matplotlib
import numpy 
import argparse
from pymavlink import mavutil
import sys 

from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

v3_x=0
v3_y=0
yawn3=0
lat3=0
lon3=0
alt_rel3=0
distancia3=0

v2_x=0
v2_y=0
yawn2=0
lat2=0
lon2=0
alt_rel2=0
distancia2=0
v1_x=0
v1_y=0
yawn1=0
lat1=0
lon1=0
alt_rel1=0
distancia1=0
erroryaw12=0
erroryaw13=0

def recibir(nothing):
    while True:
        j=sock.recvfrom(1024)
        m,ad=j
        n=pickle.loads(m)

        if type(n)==int:
            global llave
            llave=1
        elif j[1]==('127.0.0.1', 5006):
            #print('UAV 2')
            data2,addr2=j
            d2=pickle.loads(data2)
            if type(d2)==list:
                v2=d2
                global v2_x
                global v2_y
                v2_x=v2[0]                
                v2_y=v2[1]
                #print('velocidad:',v1_x,v1_y,type(v1_x),type(v1_y))
            elif type(d2)==float:
                #print('yawn1',d1,type(d1))
                global yawn2
                yawn2=d2
                global erroryaw12
                erroryaw12=d2          
            else:
                global lat2
                global lon2
                global alt_rel2
                [lat2, lon2, alt_rel2]=lat_lon_alt(d2)
                global distancia2
                distancia2=math.sqrt(lat2*lat2+lon2*lon2)
            
        elif j[1]==('127.0.0.1', 5005):
            #print('UAV 1')
            data1,addr1=j
            d1=pickle.loads(data1)
            if type(d1)==list:
                v1=d1
                global v1_x
                global v1_y
                v1_x=v1[0]                
                v1_y=v1[1]
                #print('velocidad:',v1_x,v1_y,type(v1_x),type(v1_y))
            elif type(d1)==float:
                #print('yawn1',d1,type(d1))
                global yawn1
                yawn1=d1
                '''
            elif type(d1)==type(vehicle.location.global_relative_frame):
                #print(d1)
                global pos1
                pos1=d1
            elif type(d1)==type(vehicle.home_location):
                #print(d1,j[1])
                global home1
                home1=d1
                '''
            else:
                global lat1
                global lon1
                global alt_rel1
                [lat1, lon1, alt_rel1]=lat_lon_alt(d1)
                global distancia1
                distancia1=math.sqrt(lat1*lat1+lon1*lon1)
                #print('UAV1',distancia1)                            
        elif j[1]==('127.0.0.1', 5008):
            #print('UAV 3')
            data3,addr3=j
            d3=pickle.loads(data3)
            if type(d3)==list:
                v3=d3
                global v3_x
                global v3_y
                v3_x=v3[0]                
                v3_y=v3[1]
                #print('velocidad:',v1_x,v1_y,type(v1_x),type(v1_y))
            elif type(d3)==float:
                #print('yawn1',d1,type(d1))
                global yawn3
                yawn3=d3
                global erroryaw13
                erroryaw13=d3                
                '''
            elif type(d1)==type(vehicle.location.global_relative_frame):
                #print(d1)
                global pos1
                pos1=d1
            elif type(d1)==type(vehicle.location.global_relative_frame):
                #print(d1)
                global pos1
                pos1=d1
            elif type(d1)==type(vehicle.home_location):
                #print(d1,j[1])
                global home1
                home1=d1
                '''
            else:
                global lat3
                global lon3
                global alt_rel3
                [lat3, lon3, alt_rel3]=lat_lon_alt(d3)
                global distancia3
                distancia1=math.sqrt(lat3*lat3+lon3*lon3)
                #print('UAV1',distancia1)

def lat_lon_alt(nothing):
    cadena=str(nothing)
    cad=cadena.split(',')
    orden=[i.replace('=',',')for i in (cad)]
    z=','.join(orden)
    lista=z.split(',')
    lat=float(lista[1])
    lon=float(lista[3])
    alt_rel=float(lista[5])
    return lat, lon, alt_rel



#The code came from: https://stackoverflow.com/questions/35145555/python-real-time-plotting-ros-data
#documentation about "self": https://www.w3schools.com/python/gloss_python_self.asp 


class Visualiser:
    def __init__(self):
        self.fig, (self.ax0,self.ax1,self.ax2) = plt.subplots(3)
        self.ln0, = self.ax0.plot([], [], 'r.',linestyle='solid' )
        self.ln1, = self.ax1.plot([], [], 'r.',linestyle='solid' )
        self.ln2, = self.ax2.plot([], [], 'r.',linestyle='solid' )
        self.x_data, self.y_data0, self.y_data1, self.y_data2 = [] , [] , [] , []

    def plot_init(self):
        #viento
        '''
        self.ax0.set_xlim(0, 1000)
        self.ax0.set_ylim(-100, 100)
        self.ax0.set_title('Vel Direccion x')
        self.ax1.set_xlim(0, 1000)
        self.ax1.set_ylim(-100, 100)
        self.ax1.set_title('Vel Direccion y')
        self.ax2.set_xlim(0, 1000)
        self.ax2.set_ylim(-100, 100)
        self.ax2.set_title('Vel Direccion z')
        '''
        #Error de distancia 
        '''
        self.ax0.set_xlim(0, 150)
        self.ax0.set_ylim(-6/3, 6/3)
        self.ax0.set_title('Drones 1-3')
        self.ax0.set_xlabel("Tiempo [s]")
        self.ax0.set_ylabel("Error de distancia [m]")    
        self.ax1.set_xlim(0, 150)
        self.ax1.set_ylim(-6/3, 6/3)
        self.ax1.set_title('Drones 1-2')
        self.ax1.set_xlabel("Tiempo [s]")
        self.ax1.set_ylabel("Error de distancia [m]")    
        self.ax2.set_xlim(0, 150)
        self.ax2.set_ylim(-10, 10)
        self.ax2.set_title('Rapidez ')
        self.ax2.set_xlabel("Tiempo [s]")
        self.ax2.set_ylabel("Rapidez [m/s]")

        #Error de posicion 
        self.ax0.set_xlim(0, 150)
        self.ax0.set_ylim(-6/2, 6/2)
        self.ax0.set_title('Direccion x ')
        self.ax0.set_xlabel("Tiempo [s]")
        self.ax0.set_ylabel("Error de posicion [m]")    
        self.ax1.set_xlim(0, 150)
        self.ax1.set_ylim(-6/2, 6/2)
        self.ax1.set_title('Direccion y')
        self.ax1.set_xlabel("Tiempo [s]")
        self.ax1.set_ylabel("Error de posicion [m]")    
        self.ax2.set_xlim(0, 150)
        self.ax2.set_ylim(-10, 10)
        self.ax2.set_title('Rapidez ')
        self.ax2.set_xlabel("Tiempo [s]")
        self.ax2.set_ylabel("Rapidez [m/s]")        
        #Comparacion de Yaw 
        '''
        self.ax0.set_xlim(0, 150)
        self.ax0.set_ylim(-6/3, 6/3)
        self.ax0.set_title('Yaw drone 1')
        self.ax0.set_xlabel("Tiempo [s]")
        self.ax0.set_ylabel("Yaw [rad]")    
        self.ax1.set_xlim(0, 150)
        self.ax1.set_ylim(-6/3, 6/3)
        self.ax1.set_title('Diferencia de yaw de configuracion drones 1-2')
        self.ax1.set_xlabel("Tiempo [s]")
        self.ax1.set_ylabel("Error yaw [rad]")    
        self.ax2.set_xlim(0, 150)
        #self.ax2.set_ylim(-10, 10)
        self.ax2.set_ylim(-2, 2)
        self.ax2.set_title('Diferencia de yaw de configuracion  drones 1-3')
        self.ax2.set_xlabel("Tiempo [s]")
        self.ax2.set_ylabel("Error yaw [rad]") 


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
        return self.ln0
    
    def getPOS(self, pose):
        elvector = (pose.position.x, pose.position.y, pose.position.z)
        #euler = tf.transformations.euler_from_quaternion(quaternion)
        POS_x = elvector[0] 
        POS_y = elvector[1]
        POS_z = elvector[2]          
        return (POS_x,POS_y,POS_z)
    def getVEL(self, twist):
        vector = (twist.linear.x, twist.linear.y, twist.linear.z)
        VEL_x = vector[0] 
        VEL_y = vector[1]
        VEL_z = vector[2]           
        return (VEL_x,VEL_y,VEL_z)
    def getAttitude(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        [roll, pitch, yaw] = tf.transformations.euler_from_quaternion(quaternion)         
        return (roll,pitch,yaw)
#    def odom_callback(self, msg1, msg2,msg3,odom1,odom2,odom3):
    def odom_callback(self,odom1,odom2,odom3):
        #posicion1= self.getPOS(msg1.pose.pose)
        #posicion2= self.getPOS(msg2.pose.pose)
        #posicion3= self.getPOS(msg3.pose.pose)
        posicion1= self.getPOS(odom1.pose.pose)
        posicion2= self.getPOS(odom2.pose.pose)
        posicion3= self.getPOS(odom3.pose.pose)
        [roll1, pitch1, yaw1]= self.getAttitude(odom1.pose.pose)
        [roll2, pitch2, yaw2]= self.getAttitude(odom2.pose.pose)
        [roll3, pitch3, yaw3]= self.getAttitude(odom3.pose.pose)
        #print('yaw',yaw1,yaw2,yaw3, yawn1, yawn2, yawn3)
        #posicion=(posicion1[0]-posicion2[0],posicion1[1]-posicion2[1],posicion1[2]-posicion2[2])
        #print('geometricax:',posicion1[0],lat1,"geometricay",posicion1[1],lon1,'geometricaz',posicion1[2],alt_rel2 )
        error_x=posicion1[0]-lat1
        error_y=posicion1[1]-lon1*-1
        error_z=posicion1[2]-alt_rel2
        error_yaw12=yawn1-yawn2
        error_yaw13=yawn1-yawn3
        error_distancia=math.sqrt(error_x**2+error_y**2)
        rapidez=math.sqrt(v1_x**2+v1_y**2)
        d13=math.sqrt((lat3-5*0-lat1)**2+(lon3-lon1)**2)
        d12=math.sqrt((lat2+5*0-lat1)**2+(lon2-lon1)**2)
        p13=math.sqrt((posicion3[0]-posicion1[0])**2+(posicion3[1]-posicion1[1])**2)
        p12=math.sqrt((posicion2[0]-posicion1[0])**2+(posicion2[1]-posicion1[1])**2)
        if d13==0:
            error13=0
            error12=0
        else:
            error13=d13-p13
            error12=d12-p12

        #print('error_x',error_x,'error_y',error_y, 'errordistancia', error_distancia)
        #print('d13',d13,'p13',p13,'d13-p13',error13)
        #print('[lat1,lat2]=',lat1,lat2,'[lon1,lon2]=',lon1,lon2, '[d12,p12]',d12,p12,'d12-p12',error13 )
        #velocidad= self.getVEL(msg.twist.twist)
        #self.y_data0.append(posicion[0])
        #self.y_data1.append(posicion[1])
        #self.y_data2.append(posicion[2])
        #self.y_data0.append(error_x)
        #self.y_data1.append(error_y)
        #self.y_data0.append(error13)
        #self.y_data1.append(error12)
        self.y_data0.append(yawn1)
        #self.y_data1.append(error_yaw12)
        #self.y_data2.append(error_yaw13)
        self.y_data1.append(erroryaw12)
        self.y_data2.append(erroryaw13)
        #self.y_data2.append(rapidez)

        if len(self.x_data)>2:
            estandar=st.stdev(self.y_data0)
            estandar12=st.stdev(self.y_data1)
            estandar13=st.stdev(self.y_data2)
            media=st.mean(self.y_data0)
            media12=st.mean(self.y_data1)
            media13=st.mean(self.y_data2)
            print("stddev muestral:",estandar,"max:", max(self.y_data0),"min:",min(self.y_data0),"mean:", media, 'tiempo[s]:',float(len(self.x_data))/10+1/10000)
            print("stddev muestral12:",estandar12,"max:", max(self.y_data1),"min:",min(self.y_data1),"mean:", media12)
            print("stddev muestral13:",estandar13,"max:", max(self.y_data2),"min:",min(self.y_data2),"mean:", media13)                 
        #self.y_data0.append(velocidad[0])
        #self.y_data1.append(velocidad[1])
        #self.y_data2.append(velocidad[2])
        x_index = float(len(self.x_data))/10
        self.x_data.append(x_index+1/10000)
    
    def update_plot(self, frame):
        self.ln0.set_data(self.x_data, self.y_data0)
        self.ln1.set_data(self.x_data, self.y_data1)
        self.ln2.set_data(self.x_data, self.y_data2)
        return self.ln0
'''
def callback(image, camera_info):
  # Solve all of perception here...
'''
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
address=('127.0.0.1',5010)
sock.bind(address)
if __name__=='__main__':
    thread1=threading.Thread(target=recibir, args=('no hay',))
    thread1.daemon=True
    thread1.start()
rospy.init_node('listener1')
vis = Visualiser()
#sub = rospy.Subscriber('mavros/global_position/local', Odometry, vis.odom_callback)

#sub = rospy.Subscriber('wind_state', Odometry, vis.odom_callback)

#sub = message_filters.Subscriber("wind_state", Odometry)
#sub.registerCallback(vis.odom_callback)

drone1_sub = message_filters.Subscriber('drone1/realilinkpose', Odometry)
drone2_sub = message_filters.Subscriber('drone2/realilinkpose', Odometry)
drone3_sub = message_filters.Subscriber('drone3/realilinkpose', Odometry)

odom1_sub = message_filters.Subscriber('/drone1/odom', Odometry)
odom2_sub = message_filters.Subscriber('/drone2/odom', Odometry)
odom3_sub = message_filters.Subscriber('/drone3/odom', Odometry)

#ts = message_filters.TimeSynchronizer([drone1_sub, drone2_sub,drone3_sub, odom1_sub, odom2_sub,odom3_sub], 10)
ts = message_filters.TimeSynchronizer([drone1_sub, drone2_sub,drone3_sub], 10)
#ts = message_filters.TimeSynchronizer([odom1_sub, odom2_sub,odom3_sub], 10)
ts.registerCallback(vis.odom_callback)




#sub = rospy.Subscriber('realilinkpose', Odometry, vis.odom_callback)
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)

plt.subplots_adjust(bottom=0.1, wspace=0.02, hspace=0.50)
plt.show(block=True) 



'''

                                                  Y_drone   A   x_Gazebo
                                                            |
                                                            |
                                                            |
                                                            |
                                                            |
                                                            |
                                                            |                     
                                                            |
                                                            |
                                                            |.... 
                                                            |         .  
                                                            |             .   
                                                            |                .
                                                            |                 .
                                                            |     alpha        .
                                                            |                  .
                                                            |                  V
            <---------------------------------------------- |--------------------------------------------------->
              y_Gazebo                                                                                                  X_drone
              
alpha=desfase del drone con respecto al heading del drone 1 


              '''