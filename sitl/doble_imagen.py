#!/usr/bin/env python 

#https://www.youtube.com/watch?v=Tj4zEX_pdUg   video de deteccion de color
#https://www.youtube.com/watch?v=Fchzk1lDt7Q   video de bounding box 
#https://www.youtube.com/watch?v=ukGa74saFfM   video de centroide 
#https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html contornos


import requests 
import cv2
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
import message_filters ### ADD THIS
import roslib
import sys
import rospy
from geometry_msgs.msg import Twist
import sensor_msgs
from sensor_msgs.msg import Image
import math
from std_msgs.msg import String
from std_msgs.msg import Float64
def empty(a):
	pass

#se asigna el nombre barras, puede ser cualquier otro nombre 
cv2.namedWindow("barras")
cv2.resizeWindow("barras",640,240)
cv2.createTrackbar("HUE Min","barras",5,179,empty)
cv2.createTrackbar("HUE Max","barras",31,179,empty)
cv2.createTrackbar("SAT Min","barras",11,255,empty)
cv2.createTrackbar("SAT Max","barras",255,255,empty)
cv2.createTrackbar("VALUE Min","barras",0,255,empty)
cv2.createTrackbar("VALUE Max","barras",255,255,empty)
cv2.createTrackbar("Threshold1","barras",107,255,empty)
cv2.createTrackbar("Threshold2","barras",113,255,empty)
cv2.createTrackbar("Area","barras",150,30000,empty)

'''
cv2.namedWindow("barras")
cv2.resizeWindow("barras",640,240)
cv2.createTrackbar("HUE Min","barras",1,179,empty)
cv2.createTrackbar("HUE Max","barras",31,179,empty)
cv2.createTrackbar("SAT Min","barras",11,255,empty)
cv2.createTrackbar("SAT Max","barras",255,255,empty)
cv2.createTrackbar("VALUE Min","barras",0,255,empty)
cv2.createTrackbar("VALUE Max","barras",255,255,empty)
cv2.createTrackbar("Threshold1","barras",107,255,empty)
cv2.createTrackbar("Threshold2","barras",113,255,empty)
cv2.createTrackbar("Area","barras",150,30000,empty)
'''


def stackImages(scale,imgArray):
	rows=len(imgArray)
	cols=len(imgArray[0])
	rowsAvaliable=isinstance(imgArray[0],list)
	width=imgArray[0][0].shape[1]
	height=imgArray[0][0].shape[0]
	if rowsAvaliable:
		for x in range(0,rows):
			for y in range(0,cols):
				if imgArray[x][y].shape[:2]==imgArray[0][0].shape[:2]:
					imgArray[x][y]=cv2.resize(imgArray[x][y], (0,0), None,scale,scale)
				else:
					imgArray[x][y]=cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1],imgArray[0][0].shape[0]), None,scale,scale)
				if len(imgArray[x][y].shape)==2: imgArray[x][y]=cv2.cvtColor(imgArray[x][y],cv2.COLOR_GRAY2BGR)
		imageBlank=np.zeros((height,width,3),np.uint8)
		hor=[imageBlank]*rows
		hor_con = [imageBlank]*rows
		for x in range(0,rows):
			hor[x]=np.hstack(imgArray[x])
		ver=np.vstack(hor)					  
	else:
		for x in range(0,rows):
			if imgArray[x].shape[:2]==imgArray[0].shape[:2]:
				imgArray[x]=cv2.resize(imgArray[x],(0,0),None,scale,scale)
			else:
				imgArray[x]=cv2.resize(imgArray[x],(imgArray[0].shape[1],imgArray[0].shape[0]), None, scale, scale)
			if len(imgArray[x].shape)==2: imgArray[x]=cv2.cvtColor(imgArray[x],cv2.COLOR_GRAY2BGR)
		hor=np.hstack(imgArray)
		ver=hor
	return ver	


def getcontours(img,imgContour):
	contours, hierarchy=cv2.findContours(img,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]


	for cnt in contours:
		area=cv2.contourArea(cnt)
		areaMin=cv2.getTrackbarPos("Area","barras")
		print("area=",area,"areaMin=",areaMin)

		if area>areaMin:
			cv2.drawContours(imgContour, contours, -1,(255,0,255),5)
			M = cv2.moments(cnt)
			#print M
			try:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				print(cx,cy, type(cx))
				return [cx,cy]
			except ZeroDivisionError:
				cx,cy=640,480
				return [cx,cy]
			#print(cx,cy, type(cx))
			radius=10
			cv2.circle(imgContour,(int(cx),int(cy)),radius,(0,0,255),-1)

			#peri=cv2.arcLength(cnt,True)
			#approx=cv2.approxPolyDP(cnt, 0.02*peri,True)
			#print(len(approx))
			#print("area=",area,"areaMin=",areaMin)
			#x,y,w,h=cv2.boundingRect(approx)
			#cv2.rectangle(imgContour, (x,y), (x+h,y+h),(0,255,0),5);
	






class LineFollower(object):
	def __init__(self):
		self.bridge_object=CvBridge()
		self.image_sub1=message_filters.Subscriber("/webcam1/image_raw",Image)		
		self.image_sub2=message_filters.Subscriber("/webcam2/image_raw",Image)
		self.pub=rospy.Publisher('chatter', Float64, queue_size=10)

	def camera_callback(self,data1,data2):
		try: 
			#WE select BGR because it's opencv encoding by default
			cv_image1 =self.bridge_object.imgmsg_to_cv2(data1, desired_encoding="bgr8")
			cv_image2 =self.bridge_object.imgmsg_to_cv2(data2, desired_encoding="bgr8")  
		except CvBridgeError as e:
			print(e)
		kernel=np.ones((5,5),np.uint8)
		imgContour1=cv_image1.copy()
		imgContour2=cv_image2.copy()
		imgHsv1=cv2.cvtColor(cv_image1,cv2.COLOR_BGR2HSV)
		imgHsv2=cv2.cvtColor(cv_image2,cv2.COLOR_BGR2HSV)
		h_min=cv2.getTrackbarPos("HUE Min","barras")
		h_max=cv2.getTrackbarPos("HUE Max","barras")
		s_min=cv2.getTrackbarPos("SAT Min","barras")
		s_max=cv2.getTrackbarPos("SAT Max","barras")
		v_min=cv2.getTrackbarPos("VALUE Min","barras")
		v_max=cv2.getTrackbarPos("VALUE Max","barras")
		#print(h_min)

		lower=np.array([h_min,s_min,v_min])
		upper=np.array([h_max,s_max,v_max])
		mask1=cv2.inRange(imgHsv1,lower,upper)
		mask2=cv2.inRange(imgHsv2,lower,upper)
		result1=cv2.bitwise_and(cv_image1,cv_image1,mask=mask1)
		result2=cv2.bitwise_and(cv_image2,cv_image2,mask=mask2)

		imgBlur1=cv2.GaussianBlur(result1,(7,7),1)
		imgBlur2=cv2.GaussianBlur(result2,(7,7),1)
		imgGray1=cv2.cvtColor(imgBlur1,cv2.COLOR_BGR2GRAY)
		imgGray2=cv2.cvtColor(imgBlur2,cv2.COLOR_BGR2GRAY)
		Threshold1=cv2.getTrackbarPos("Threshold1","barras")
		Threshold2=cv2.getTrackbarPos("Threshold2","barras")
		imgCanny1=cv2.Canny(imgGray1,Threshold1,Threshold2)
		imgCanny2=cv2.Canny(imgGray2,Threshold1,Threshold2)
		imgDilation1=cv2.dilate(imgCanny1,kernel, iterations=1)
		imgDilation2=cv2.dilate(imgCanny2,kernel, iterations=1)
		centro1=getcontours(imgDilation1,imgContour1) 
		centro2=getcontours(imgDilation2,imgContour2)
		
		d_teoricax=640/math.tan(0.25/2)
		if type(centro1)==list and type(centro2)==list:
			tanbeta1=(centro1[0]-640)/d_teoricax
			tanbeta2=(centro2[0]-640)/d_teoricax
			beta1=np.arctan(tanbeta1)
			beta2=np.arctan(tanbeta2)
			A=math.pi/2-abs(beta1)
			B=math.pi/2-abs(beta2)
			d_camaras=2
			#d_objeto=d_camaras*(math.tan(A)*math.tan(B))/(math.tan(A)+math.tan(B))
			#d_objeto=d_camaras*(1/(1/math.tan(A)+1/math.tan(B)))
			if abs(A)>math.pi/2:
				aux1=math.pi-abs(A)
			else:
				aux1=A
			if abs(B)>math.pi/2:
				aux2=math.pi-abs(B)
			else:
				aux2=B
			C=math.pi-abs(aux1)-abs(aux2)
			d2=d_camaras*math.sin(aux1)/math.sin(C)
			d22=d2*math.sin(aux2)
			d_objeto=d_camaras*(1/(1/math.tan(A)+1/math.tan(B)))
			print('centro1',centro1,'centro2',centro2,'data type',type(centro1),'d_objeto',d_objeto,d22,'beta1',beta1,'beta2',beta2)
			#hello_str = "hello world %s" % rospy.get_time()  #inforamcion de una string y el tiempo
			
			self.pub.publish(d_objeto) #publica iformacion en el topico
		else:
			print(type(centro1),type(centro2))




		
		#hstack=np.hstack([cv_image,result])
		StackedImages=stackImages(0.25,[cv_image1,result1,imgDilation1,imgContour1])

		#cv2.circle(StackedImages,(int(cx),int(cy)),10,(0,255,0),-1)

		#cv2.imshow("Androidcam",cv_image)
	#	cv2.imshow("HSV",imgHsv)
		#cv2.imshow("mask",mask)
		#cv2.imshow("result",result)
		cv2.imshow("Horizontal stack",StackedImages)

		#cv2.imshow("Image_window",cv_image)
		#cv2.imshow("Image_HSV",imgHsv)
		cv2.imshow("Image_HSV1",imgContour1)
		cv2.imshow("Image_HSV2",imgContour2)
		cv2.waitKey(1)


def main():
	line_follower_object=LineFollower()
	rospy.init_node("line_following_node",anonymous=True)

	try:
		
		ts = message_filters.ApproximateTimeSynchronizer([line_follower_object.image_sub1,line_follower_object.image_sub2],10,0.1)
		ts.registerCallback(line_follower_object.camera_callback)
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting it down")
	cv2.destroyAllWindows()
if __name__ == '__main__':
  	main()  
