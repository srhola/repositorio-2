#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
#https://dronekit-python.readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
#####################dependencies###########################
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, LocationLocal,  LocationGlobal
import time
import socket
import exceptions
import math
import matplotlib
import numpy 
import argparse
from pymavlink import mavutil
import socket
import pickle
import threading 
import sys 
import rospy
import message_filters
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
########################functions##############################
#funcion para recibir data
def arcotangente(opuesto,adyacente):
    if adyacente<0:
        angulo=math.atan(opuesto/adyacente)+math.pi
    else:
        angulo=math.atan(opuesto/adyacente)
    return angulo
def callback(msg):
    u=msg.pose.pose.position.x
    v=msg.pose.pose.position.y
    rospy.loginfo('x2:{},y2:{}'.format(u,v))
def callback1(msg):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    x1=msg.pose.pose.position.x
    y1=msg.pose.pose.position.y
    #tx1=type(x)
    #ty1=type(y)    
    #rospy.loginfo('x1:{},y1:{}'.format(x1,y1))
    
    #rospy.loginfo('type x:{}, type y:{}'.format(tx,ty))
def listener1():
    rospy.init_node('listenerdrone1', anonymous=False)
    #rospy.Subscriber("drone1/mavros/global_position/local", Odometry, callback)
    sub = message_filters.Subscriber("drone2/data_de_drone1", Odometry)
    sub.registerCallback(callback)
    rospy.spin()

    #rospy.Subscriber("drone1/mavros/global_position/local", Odometry, callback1)
    #rospy.Subscriber("drone2/mavros/global_position/local", Odometry, callback2)
    #rospy.Subscriber("drone3/mavros/global_position/local", Odometry, callback3)
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

def subscriber():
    rospy.init_node('listenerdrone1', anonymous=False)
    #sub=rospy.Subscriber("camera/color/image_raw",Image,msg_receiver)
    #sub=rospy.Subscriber("wind_state",Odometry,callback)
    #sub=rospy.Subscriber("mavros/global_position/local",Odometry,callback)
    rospy.Subscriber("drone2/realilinkpose",Odometry,callback1)
    #rospy.Subscriber("drone1/mavros/global_position/compass_hdg",Float64,callback2)
    #rospy.Subscriber("drone1/mavros/global_position/local", Odometry, callback1)
    v_x=0
    v_y=0
    N=1+v_x-1
    E=2+v_y-2
    counter1=0

    x = 1.0
    y = 0.0
    z=0.0
    pitch=0
    yaw=1.57
    roll=0
    vx = 0.1
    vy = -0.1
    vz=0.0
    v_angx=0.1
    v_angy=0.0
    v_angz = 0.1

    [hlat1,hlon1,halt_rel1]=lat_lon_alt(vehicle.home_location)
    while counter1<60:
        send_global_ned_velocity(N,E,0)
        #print(vehicle.location.global_relative_frame, type(vehicle.location.global_relative_frame),"hlat1",hlat1,"hlon",hlon1)
        posicionglobal1=vehicle.location.global_relative_frame
        condition_yaw(315)
        counter1=counter1+0.1
        print counter1
        #publica odometria
        [x,y,z]=lat_lon_alt(vehicle.location.local_frame)
        [vx, vy, vz]=vehicle.velocity
        [pitch,yaw,roll]=lat_lon_alt(vehicle.attitude)
        [v_angx,v_angy,v_angz]=[0,0,0]
        odometria(x,y,z,roll,pitch,yaw,vx,vy,vz,v_angx,v_angy,v_angz)
        print(" GPS: %s" % vehicle.gps_0.epv)
        #print ('Groundspeed: %s', vehicle.velocity)   
        #print ('POSITION: %s'% vehicle.location.global_relative_frame)
        #msg=pickle.dumps(vehicle.location.global_relative_frame)
        envio()
        time.sleep(0.1)
    dN=80
    dE=0
    goto2(dN,dE,0,goto_position_target_global_int)
    time.sleep(10)
    goto2(10,20,0,goto_position_target_global_int)
    time.sleep(10)    
    goto2(-90,-20,0,goto_position_target_global_int)   
    vehicle.mode=VehicleMode('RTL')
    rospy.spin()

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
                #print(v2)
            elif type(d2)==float:
                print('yaw2:',d2,type(d2))
                global yawn2
                yawn2=d2
            elif type(n)==type(vehicle.location.global_relative_frame):
                print(type(vehicle.location.global_relative_frame),j[1])
            else:
                global lat2
                global lon2
                global alt_rel2
                [lat2, lon2, alt_rel2]=lat_lon_alt(d2)
                global distancia2
                distancia2=math.sqrt(lat2*lat2+lon2*lon2)
                print('Distancia2:',distancia2)
            
        elif j[1]==('127.0.0.1', 5008):
            #print('UAV 3')
            data3,addr3=j
            d3=pickle.loads(data3)
            if type(d3)==list:
                v3=d3
                #print(v3)
            elif type(d3)==float:
                print('yaw3:',d3,type(d3))
                global yawn3
                yawn3=d3
            elif type(n)==type(vehicle.location.global_relative_frame):
                print(type(vehicle.location.global_relative_frame),j[1])
            else:
                
                global lat3
                global lon3
                global alt_rel3
                [lat3, lon3, alt_rel3]=lat_lon_alt(d3)
                global distancia3
                distancia3=math.sqrt(lat3*lat3+lon3*lon3)
                print("Distancia3:",distancia3)



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


def turbo(distancia,k,d_uav1,d_uav2):
    if distancia!=10: #si la distancia es 10 nada ha cambiado
        if distancia!=k: 
            #print('distancia:',distancia)
            print('distancia entre uavs 1-2',d_uav1,'distancia entre uavs 1-3',d_uav2)
        else: 
            print(distancia,k) 
    else:
        pass
    k=distancia

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string=args.connect
    if not connection_string:
        import dronekit_sitl
        sitl=dronekit_sitl.start_default()
        connection_string=sitl.connection_string()

    vehicle = connect(connection_string,wait_ready=True)
    return vehicle
def connectMyCopter2():

    connection_string= '127.0.0.1:14551'
    vehicle = connect(connection_string,wait_ready=True)
    return vehicle
vehicle = connectMyCopter2()
#Function to arm  the drone and takeoff into the air
def arm_and_takeoff(aTargetAltitude):
    while not vehicle.is_armable:
        print('waiting for the vehicle to be armable')
        time.sleep(1)
    #switch vehicle to GUIDED MODE
    vehicle.mode=VehicleMode('GUIDED')
    while vehicle.mode !='GUIDED':
        print('waiting for the vehicle to enter GUIDED mode')
        time.sleep(1)
    #ARM VEHICLE ONCE GUIDED MODE IS CONFIRMED
    vehicle.armed=True
    while vehicle.armed==False:
        print('waiting for vehicle to be armed.')
        time.sleep(1)
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print('current altitude: %d'%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*.95:
            break
        time.sleep(1)
    print('target altitude reached')
    return None
def send_local_ned_velocity(vx, vy, vz):
    msg=vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,#BITMASK----> consider only the velocities
        0, 0, 0,             #POSITION
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
def send_global_ned_velocity(vx, vy, vz):
    msg=vehicle.message_factory.set_position_target_global_int_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111000111,#BITMASK----> consider only the velocities
        0, 0, 0,             #POSITION
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def goto(dNorth, dEast,gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        envio()
        if remainingDistance<=0.2: #Just below target, in case of undershoot.
            print("Reached target")
            
            break;
        time.sleep(2)
def goto2(dNorth, dEast, heading, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        condition_yaw(heading)
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        envio()
        [x,y,z]=lat_lon_alt(vehicle.location.local_frame)
        [vx, vy, vz]=vehicle.velocity
        [pitch,yaw,roll]=lat_lon_alt(vehicle.attitude)
        [v_angx,v_angy,v_angz]=[0,0,0]

        odometria(x,y,z,roll,pitch,yaw,vx,vy,vz,v_angx,v_angy,v_angz)
        #if remainingDistance<=targetDistance*0.001: #Just below target, in case of undershoot.
        if remainingDistance<=0.2: #Just below target, in case of undershoot.        
            print("Reached target")
            break;
        time.sleep(0.1)
def envio():
    msg1=pickle.dumps(vehicle.velocity)
    sock.sendto(msg1,('127.0.0.1',5009))
    sock.sendto(msg1,('127.0.0.1',5006))
    sock.sendto(msg1,('127.0.0.1',5008))
    sock.sendto(msg1,('127.0.0.1',5010))
    msg2=pickle.dumps(vehicle.location.local_frame)
    sock.sendto(msg2,('127.0.0.1',5009))
    sock.sendto(msg2,('127.0.0.1',5006))
    sock.sendto(msg2,('127.0.0.1',5008))
    sock.sendto(msg2,('127.0.0.1',5010))
    [the_pitch,the_yaw,the_roll]=lat_lon_alt(vehicle.attitude)
    print('yaw=',the_yaw, type(the_yaw) )
    send_yaw=pickle.dumps(the_yaw)
    sock.sendto(send_yaw,('127.0.0.1',5009))
    sock.sendto(send_yaw,('127.0.0.1',5006))
    sock.sendto(send_yaw,('127.0.0.1',5008))
    sock.sendto(send_yaw,('127.0.0.1',5010))          
    msg3=pickle.dumps(vehicle.location.global_relative_frame)
    sock.sendto(msg3,('127.0.0.1',5009))
    sock.sendto(msg3,('127.0.0.1',5006))
    sock.sendto(msg3,('127.0.0.1',5008))
    msg4=pickle.dumps(vehicle.home_location)
    sock.sendto(msg4,('127.0.0.1',5009))
    sock.sendto(msg4,('127.0.0.1',5006))
    sock.sendto(msg4,('127.0.0.1',5008))
    
    actitud=lat_lon_alt(vehicle.attitude)
    [bitch,yawn,rolls]=actitud
    actitud2=[1,bitch,yawn,rolls]
    envio_actitud=pickle.dumps(actitud2)
    sock.sendto(envio_actitud,('127.0.0.1',5010))


def odometria(x,y,z,roll,pitch,yaw,vx,vy,vz,v_angx,v_angy,v_angz):
    odom_pub = rospy.Publisher("drone1/odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    current_time = rospy.Time.now()
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(roll,pitch , yaw)
    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, z),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )
    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    # set the position
    odom.pose.pose = Pose(Point(x, y, z), Quaternion(*odom_quat))
    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(v_angx, v_angy, v_angz))
    # publish the message
    odom_pub.publish(odom)
    last_time = current_time

##############MAIN EXEUTABLE ################################
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
address=('127.0.0.1',5005)
sock.bind(address)
r=0
i=0
k=0
distancia1=30
distancia2=20
distancia3=20
lon1=0
lon2=0
lon3=0
lat1=0
lat2=0
lat3=0
llave=1

if __name__=='__main__':

    #rospy.sleep(1.)
    thread1=threading.Thread(target=recibir, args=('no hay',))
    thread1.daemon=True
    thread1.start()
    try:
        arm_and_takeoff(11)
        msg4=pickle.dumps(vehicle.home_location)
        sock.sendto(msg4,('127.0.0.1',5009))
        sock.sendto(msg4,('127.0.0.1',5006))
        sock.sendto(msg4,('127.0.0.1',5008))
        time.sleep(1)
        #send_local_ned_velocity(1,0,0)
        time.sleep(1)
        subscriber()

    except rospy.ROSInterruptException:
        pass


  
