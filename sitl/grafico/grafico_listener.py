#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import random
from itertools import count
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading 
import sys 	





###############################callbacks###############################

def callback1(msg):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    x1=msg.pose.pose.position.x
    y1=msg.pose.pose.position.y
    #tx1=type(x)
    #ty1=type(y)
    rospy.loginfo('x1:{},y1:{}'.format(x1,y1))
    #rospy.loginfo('type x:{}, type y:{}'.format(tx,ty))

def callback2(msg):
    x2=msg.pose.pose.position.x
    y2=msg.pose.pose.position.y
    rospy.loginfo('x2:{},y2:{}'.format(x2,y2))

def callback3(msg):
    x3=msg.pose.pose.position.x
    y3=msg.pose.pose.position.y
    rospy.loginfo('x3:{},y3:{}'.format(x3,y3))

plt.style.use('fivethirtyeight')
x_vals = []
y_vals = []
t = []
index = count()

def callback(msg):
	t.append(next(index))
	global x_val
	global y_val
	x_val=msg.pose.pose.position.x
	y_val=msg.pose.pose.position.y

	#x_vals.append(x_val)
	#y_vals.append(y_val)
	#print(x_vals,y_vals)
	

	print(x_val,y_val)
	
	#plt.cla()
	#plt.plot(x_vals,y_vals)
    return x_val y_val

def animate(msg):
	print(x_val,y_val)
	x_vals.append(x_val)
	y_vals.append(y_val)
	print(x_vals,y_vals)
	plt.cla()
	plt.plot(x_vals,y_vals)
	

#############################listener##################################

def listener1():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener1', anonymous=True)
    rospy.Subscriber("wind_state", Odometry, callback)
    #print(x_val,y_val)
    
    #ani=FuncAnimation(plt.gcf(), animate, interval=1000 )
    #plt.tight_layout()
    #plt.show()
    #rospy.Subscriber("drone1/mavros/global_position/local", Odometry, callback1)
    #rospy.Subscriber("drone2/mavros/global_position/local", Odometry, callback2)
    #rospy.Subscriber("drone3/mavros/global_position/local", Odometry, callback3)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



def recibir(msg):
	plt.show()



def idiota(msg):
	ani=FuncAnimation(plt.gcf(), animate, interval=1000 )
	plt.tight_layout()
	plt.show()




if __name__ == '__main__':
    listener1()
    #plt.show()
    #thread1=threading.Thread(target=recibir, args=('no hay',))
    #thread1.daemon=True
    #thread1.start()

    #thread1=threading.Thread(target=idiota, args=('no hay',))
    #thread1.daemon=True
    #thread1.start()
