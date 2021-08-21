#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
import pandas as pd 
from matplotlib.animation import FuncAnimation
import message_filters
import socket
import pickle
import threading 
import math
import csv
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
variable1_6=[]
variable1_7=[]
variable1_8=[]
variable1_9=[]
variable1_10=[]
variable1_11=[]

variable2_1=[]
variable2_2=[]
variable2_3=[]
variable2_4=[]
variable2_5=[]
variable2_6=[]
variable2_7=[]
variable2_8=[]
variable2_9=[]
variable2_10=[]
variable2_11=[]
variable2_12=[]
variable2_13=[]
variable2_14=[]

variable3_1=[]
variable3_2=[]
variable3_3=[]
variable3_4=[]
variable3_5=[]
variable3_6=[]
variable3_7=[]
variable3_8=[]
variable3_9=[]
variable3_10=[]
variable3_11=[]
variable3_12=[]
variable3_13=[]
variable3_14=[]
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
                
                global v1_x
                global v1_y
                if len(d1)==3:
                    v1=d1
                    v1_x=v1[0]                
                    v1_y=v1[1]
                    #print(len(d1))
                else:
                    global actitud1
                    actitud1=d1
                    #print('d1',len(actitud1))
                    

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
        self.ln0, = self.ax0.plot([], [], 'r.',markersize=5)
        self.ln1, = self.ax1.plot([], [], 'r.', markersize=5)
        self.ln00, = self.ax0.plot([], [], 'b.', markersize=1)
        self.ln11, = self.ax1.plot([], [], 'b.', markersize=1)
        self.ln2, = self.ax2.plot([], [], 'r.', markersize=1)
        self.x_data, self.y_data0, self.y_data00, self.y_data1, self.y_data11, self.y_data2 = [] , [] , [] , [], [] , []

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
        self.ax0.set_xlim(0, 140)
        self.ax0.set_ylim(-6/1, 6/1)
        self.ax0.set_title('Drones 1-3')
        self.ax0.set_xlabel("Tiempo [s]")
        self.ax0.set_ylabel("Error de distancia [m]")    
        self.ax1.set_xlim(0, 140)
        self.ax1.set_ylim(-6/1, 6/1)
        self.ax1.set_title('Drones 1-2')
        self.ax1.set_xlabel("Tiempo [s]")
        self.ax1.set_ylabel("Error de distancia [m]")    
        self.ax2.set_xlim(0, 140)
        self.ax2.set_ylim(0, 10)
        self.ax2.set_title('Rapidez drone1')
        self.ax2.set_xlabel("Tiempo [s]")
        self.ax2.set_ylabel("Rapidez [m/s]")
        '''
        '''
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
        '''
        self.ax0.set_xlim(0, 150)
        self.ax0.set_ylim(-6/3, 6/3)
        self.ax0.set_title('Yaw drone 1')
        self.ax0.set_xlabel("Tiempo [s]")
        self.ax0.set_ylabel("Yaw [rad]")    
        self.ax1.set_xlim(0, 150)
        self.ax1.set_ylim(-6/3, 6/3)
        self.ax1.set_title('Diferencia de yaw 1-2')
        self.ax1.set_xlabel("Tiempo [s]")
        self.ax1.set_ylabel("Error yaw [rad]")    
        self.ax2.set_xlim(0, 150)
        #self.ax2.set_ylim(-10, 10)
        self.ax2.set_ylim(-2, 2)
        self.ax2.set_title('Diferencia de yaw 1-3')
        self.ax2.set_xlabel("Tiempo [s]")
        self.ax2.set_ylabel("Error yaw [rad]") 
        
        self.ax0.set_xlim(0, 150)
        self.ax0.set_ylim(-1, 1)
        self.ax0.set_title('pitch')
        self.ax0.set_xlabel("Tiempo [s]")
        self.ax0.set_ylabel("pitch [rad]")    
        self.ax1.set_xlim(0, 150)
        self.ax1.set_ylim(-1, 1)
        self.ax1.set_title('roll ')
        self.ax1.set_xlabel("Tiempo [s]")
        self.ax1.set_ylabel("roll [rad]")    
        #self.ax2.set_xlim(0, 150)
        #self.ax2.set_ylim(-2, 2)
        #self.ax2.set_title('yaw')
        #self.ax2.set_xlabel("Tiempo [s]")
        #self.ax2.set_ylabel("yaw [rad]") 
        self.ax2.set_xlim(0, 150)
        self.ax2.set_ylim(-10, 10)
        self.ax2.set_title('Rapidez ')
        self.ax2.set_xlabel("Tiempo [s]")
        self.ax2.set_ylabel("Rapidez [m/s]")        
        '''   
        #posicion
        self.ax0.set_xlim(0, 150)
        self.ax0.set_ylim(-100, 100)
        self.ax0.set_title('Direccion x')
        self.ax0.set_xlabel("Tiempo [s]")
        self.ax0.set_ylabel("posicion [m]") 
        self.ax1.set_xlim(0, 150)
        self.ax1.set_ylim(-100, 100)
        self.ax1.set_title('Direccion y')
        self.ax1.set_xlabel("Tiempo [s]")
        self.ax1.set_ylabel("posicion [m]") 
        self.ax2.set_xlim(0, 150)
        self.ax2.set_ylim(-10, 10)
        self.ax2.set_title('Direccion z')
        self.ax2.set_xlabel("Tiempo [s]")
        self.ax2.set_ylabel("posicion [m]") 
    
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
        # se obteine la actitud desde Gazebo
        posicion1= self.getPOS(odom1.pose.pose)
        posicion2= self.getPOS(odom2.pose.pose)
        posicion3= self.getPOS(odom3.pose.pose)
        # se obteine la actitud desde Gazebo
        [roll1, pitch1, yaw1]= self.getAttitude(odom1.pose.pose)
        [roll2, pitch2, yaw2]= self.getAttitude(odom2.pose.pose)
        [roll3, pitch3, yaw3]= self.getAttitude(odom3.pose.pose)
        #print('yaw',yaw1,yaw2,yaw3, yawn1, yawn2, yawn3)
        print('geometricax:',posicion1[0],lat1,"geometricay",posicion1[1],lon1,'geometricaz',posicion1[2],alt_rel2 )
        error_x=posicion1[0]-lat1
        error_y=posicion1[1]-lon1*-1
        error_z=posicion1[2]-alt_rel2
        error_yaw12=yawn1-yawn2
        error_yaw13=yawn1-yawn3
        error_distancia=math.sqrt(error_x**2+error_y**2)
        rapidez=math.sqrt(v1_x**2+v1_y**2)
        #distancia entre cada drone utilizando datos provenienetes del gps simulado
        d13=math.sqrt((lat3-5*0-lat1)**2+(lon3-lon1)**2)
        d12=math.sqrt((lat2+5*0-lat1)**2+(lon2-lon1)**2)
        #distancia entre cada drone utilizando datos provenienetes de Gazebo
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
        #self.y_data0.append(yawn1)
        #self.y_data1.append(error_yaw12)
        #self.y_data2.append(error_yaw13)
        #self.y_data1.append(erroryaw12)
        #self.y_data2.append(erroryaw13)
        self.y_data0.append(posicion1[0])
        self.y_data00.append(lat1)
        self.y_data1.append(posicion1[1])
        self.y_data11.append(-lon1)
        #self.y_data2.append(yaw1)
        self.y_data2.append(rapidez)
        variable1_6.append(yaw1)
        variable1_7.append(roll1)
        variable1_8.append(pitch1)
        variable1_9.append(yawn1)
        variable1_10.append(v1_x)
        variable1_11.append(v1_y)   

        variable2_1.append(-posicion2[0])
        variable2_2.append(lat2)
        variable2_3.append(posicion2[1])
        variable2_4.append(-lon2)
        variable2_5.append(rapidez)
        variable2_6.append(yaw2)
        variable2_7.append(roll2)
        variable2_8.append(pitch2)
        variable2_9.append(yawn2)
        variable2_10.append(v2_x)
        variable2_11.append(v2_y)   
        variable2_12.append(erroryaw12)   
        variable2_13.append(p12)   
        variable2_14.append(d12)   

        variable3_1.append(-posicion3[0])
        variable3_2.append(lat3)
        variable3_3.append(posicion3[1])
        variable3_4.append(-lon3)
        variable3_5.append(rapidez)
        variable3_6.append(yaw3)
        variable3_7.append(roll3)
        variable3_8.append(pitch3)
        variable3_9.append(yawn3)
        variable3_10.append(v3_x)
        variable3_11.append(v3_y)
        variable3_12.append(erroryaw13)  
        variable3_13.append(p13)   
        variable3_14.append(d13) 
        #self.y_data3.append(v1_x)
        #self.y_data33.append(v1_y)
        #self.y_data4.append(yaw1)
        #self.y_data44.append(yawn1)            
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
        self.ln00.set_data(self.x_data, self.y_data00)
        self.ln11.set_data(self.x_data, self.y_data11)
        self.ln2.set_data(self.x_data, self.y_data2)
        return self.ln0

    def tatakae(self, eren):
        print('tatakae',self.x_data)

    def guardar_datos(self):
        with open('tutorial.csv','w') as csvfile: 
            fieldnames=['a', 'b'] 
            thewriter =csv.DictWriter(csvfile, fieldnames=fieldnames)
            thewriter.writeheader()
            #print(type(self.x_data) )
            k=self.x_data
            kk=self.y_data0
            for a in k:
                for b in kk:
                    thewriter.writerow({'a':a, 'b':b})
    def guardar_datos4(self):

        print(type(self.x_data) )
        k1=self.x_data
        k2=self.y_data0
        k3=self.y_data00
        k4=self.y_data1
        k5=self.y_data11
        k6=self.y_data2
        k7=variable1_6
        k8=variable1_7
        k9=variable1_8
        k10=variable1_9
        k11=variable1_10
        k12=variable1_11

        k13=variable2_1
        k14=variable2_2
        k15=variable2_3
        k16=variable2_4
        k17=variable2_5
        k18=variable2_6
        k19=variable2_7
        k20=variable2_8
        k21=variable2_9
        k22=variable2_10
        k23=variable2_11
        k24=variable2_12
        k25=variable2_13
        k26=variable2_14

        k27=variable3_1
        k28=variable3_2
        k29=variable3_3
        k30=variable3_4
        k31=variable3_5
        k32=variable3_6
        k33=variable3_7
        k34=variable3_8
        k35=variable3_9
        k36=variable3_10
        k37=variable3_11
        k38=variable3_12
        k39=variable3_13
        k40=variable3_14

        li=[k1,k2,k3,k4,k5,k6,k7,k8,k9,k10,k11,k12,k13,k14,k15,k16, k17,k18,k19,k20,k21,k22,k23,k24,k25,k26,k27,k28,k29,k30,k31,k32,k33,k34,k35,k36,k37,k38,k39,k40]
        data=pd.DataFrame(li)
        df=data.T
        string_vector1=['posicion1 gazebo x','posicion1 medida x','posicion1 gazebo y','posicion1 medida y','rapidez1','yaw1','roll1','pitch1','yaw_drone1','Vel1_x','Vel1_y']
        string_vector2=['posicion2 gazebo x','posicion2 medida x','posicion2 gazebo y','posicion2 medida y','rapidez2','yaw2','roll2','pitch2','yaw_drone2','Vel2_x','Vel2_y','error12','distancia_medida12','distancia_real12']
        string_vector3=['posicion3 gazebo x','posicion3 medida x','posicion3 gazebo y','posicion3 medida y','rapidez3','yaw3','roll3','pitch3','yaw_drone3','Vel3_x','Vel3_y','error13','distancia_medida13','distancia_real13']
        df.columns=['tiempo']+string_vector1+string_vector2+string_vector3
        df.to_csv('/home/fabian/Escritorio/data.csv')
        print(df)
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

#drone1_sub = message_filters.Subscriber('drone4/realilinkpose', Odometry)
#drone2_sub = message_filters.Subscriber('drone5/realilinkpose', Odometry)
#drone3_sub = message_filters.Subscriber('drone6/realilinkpose', Odometry)

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
vis.tatakae('nada')
vis.guardar_datos4()


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
#print('das finale')
