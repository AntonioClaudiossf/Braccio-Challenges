#!/usr/bin/env python               
import rospy                            #inclut les librairies python
from std_msgs.msg import Float64MultiArray       #   messages Float64MultiArray
from aruco_msgs.msg import MarkerArray
import geometry_msgs.msg
import tf
from braccio_challenge.msg import Cube
from braccio_challenge.msg import ListeCubes

import math
import numpy as np


# Creation et initialisation de message

msg_robot1 = Float64MultiArray() #definition du type de message
msg_robot2 = Float64MultiArray() #definition du type de message

msg_camera1 = Float64MultiArray() #definition du type de message
msg_camera2 = Float64MultiArray() #definition du type de message

msg_robot1.data = [0.5,0.5,2.0,1.5,1.0,0]
msg_robot2.data = [0.0,1.57,0.0,0.0,0.0,1.0]

msg_cube = ListeCubes()
un_seul_cube = Cube()
un_seul_cube.id = 42
un_seul_cube.x = 24
un_seul_cube.y = 8
un_seul_cube.z = 16

liste_cubes = ListeCubes()


num_cube = []
x_cube = []
y_cube = []
z_cube = []


def transformation(RotZ_camera,TranX_camera,TranZ_camera,RotY_camera):
    
    Rz = np.array([[-1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0,  0, 1, 0],
                   [0,  0, 0, 1]]).dot(np.array([[np.cos(RotZ_camera), -np.sin(RotZ_camera),0,0],
                                                 [np.sin(RotZ_camera),  np.cos(RotZ_camera),0,0],
                                                 [0, 0, 1, 0],
                                                 [0, 0, 0, 1]]))
    
    T = np.array([[1, 0, 0, TranX_camera],
                  [0, 1, 0, 0],
                  [0, 0, 1, TranZ_camera],
                  [0, 0, 0, 1]])

    Ry = np.array([[-1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0,  0, 1, 0],
                   [0,  0, 0, 1]]).dot(np.array([[np.cos(RotY_camera), 0, np.sin(RotY_camera),0],
                                                 [0, 1, 0, 0],
                                                 [-np.sin(RotY_camera),0, np.cos(RotY_camera),0],
                                                 [0, 0, 0, 1]]))
    
    T_final = Rz.dot(T).dot(Ry)
    return T_final






# definition de la fonction qui recoit les infos d'aruco

def callbackAruco(msg):   
    global liste_cubes
    for i in range(0, len(msg.markers)):
        cube_detected = int(msg.markers[i].id/10)
        if (cube_detected not in num_cube and cube_detected != 10):
            num_cube.append(cube_detected)
            print("id = ",msg.markers[i].id)
            #print("cubes presentes",num_cube)
            x_cube.append(msg.markers[i].pose.pose.position.x)
            y_cube.append(msg.markers[i].pose.pose.position.y)
            z_cube.append(msg.markers[i].pose.pose.position.z)
            
            x = msg.markers[i].pose.pose.position.x
            y = msg.markers[i].pose.pose.position.y
            z = msg.markers[i].pose.pose.position.z-0.05

            
            a = msg.markers[i].pose.pose.orientation.x
            b = msg.markers[i].pose.pose.orientation.y
            c = msg.markers[i].pose.pose.orientation.z
            d = msg.markers[i].pose.pose.orientation.w
            
                                       
            frame = np.array([[2*(a*a + b*b)-1, 2*(b*c - a*d),   2*(b*d + a*c),  x],
                              [2*(b*c + a*d),   2*(a*a + c*c)-1, 2*(c*d - a*b),  y],
                              [2*(b*d - a*c),  2*(c*d + a*b),   2*(a*a + d*d)-1, z],
                              [0, 0, 0, 1]])
            
           # print(frame)
            
           # T01 = Transformation(0,0,math.pi,0,0,0)*RotZ(RotZ_camera)
           # T12 = Transformation(0,0,0,TranX_camera,0,0)
            #T23 = Transformation(0,0,0,0,0,TranZ_camera)
            #T34 = Transformation(0,0,math.pi,0,0,0)*RotY(RotY_camera)
            
            position_cube = transformation(RotZ_camera,TranX_camera,TranZ_camera,RotY_camera).dot(frame)

            #print("id",cube_detected, "pos",msg.markers[i].pose.pose.position.x,msg.markers[i].pose.pose.position.y,msg.markers[i].pose.pose.position.z)
            print(position_cube)
            #print(transformation(RotZ_camera,TranX_camera,TranZ_camera,RotY_camera))
            #x_cube.append(position_cube[0])
            #y_cube.append(position_cube[1])
            #z_cube.append(position_cube[2])
            
        
    liste_cubes=  ListeCubes()
    for i in range(0,len(num_cube)):
        cube = Cube()
        cube.id = num_cube[i]
        cube.x = x_cube[i]
        cube.y = y_cube[i]
        cube.z = z_cube[i]
        #cube.roll = roll_cube[i]
        #cube.pitch = pitch_cube[i]
        #cube.yaw = yaw_cube[i]
        
        
        liste_cubes.cubes.append(cube)
    

rospy.init_node('publisher_camera')   #initialisation du noeud et affectation d'un nom


# definition de trois publishers et d'un subscriber

pub_cube = rospy.Publisher('/cube_presents', ListeCubes, queue_size=10)     
pub_robot = rospy.Publisher('/braccio_arm/joint_angles', Float64MultiArray, queue_size=10)     
pub_camera = rospy.Publisher('/braccio_arm/camera_joint_angles', Float64MultiArray, queue_size=10) 
rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, callbackAruco)
                                                        
rate = rospy.Rate(10)                       #definit la frequence d envoie 10Hz

RotZ_camera = 0.0
TranX_camera = 0.8
TranZ_camera = 0.5
RotY_camera = 0.5

listener = tf.TransformListener()

while not rospy.is_shutdown():         #tant que ros est en fonctionnement  
    
    if RotZ_camera < 2*math.pi:
        msg_camera1.data = [RotZ_camera,TranX_camera,TranZ_camera,RotY_camera]
        RotZ_camera = RotZ_camera+1
        pub_camera.publish(msg_camera1)
        rospy.sleep(2.)

    #pub_robot.publish(msg_robot2)
    rospy.sleep(2.)
    
    pub_cube.publish(liste_cubes)
    
    #rate.sleep()                       # a remettre apres exemple
    
