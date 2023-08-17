#!/usr/bin/env python               
import rospy                            #inclut les librairies python
from std_msgs.msg import Float64MultiArray       #   messages Float64MultiArray
from aruco_msgs.msg import MarkerArray

from braccio_challenge.msg import Cube
from braccio_challenge.msg import ListeCubes

import math


# Creation et initialisation de message

msg_robot1 = Float64MultiArray() #definition du type de message
msg_robot2 = Float64MultiArray() #definition du type de message

msg_camera1 = Float64MultiArray() #definition du type de message
msg_camera2 = Float64MultiArray() #definition du type de message




msg_cube = ListeCubes()
un_seul_cube = Cube()
un_seul_cube.id = 42
un_seul_cube.x = 24
un_seul_cube.y = 8
un_seul_cube.z = 16

liste_cubes = ListeCubes()

num_cube = []



# definition de la fonction qui recoit les infos d'aruco

def callbackAruco(msg):   
    global liste_cubes
    for i in range(0, len(msg.markers)):
        cube_detected = int(msg.markers[i].id/10)
        if (cube_detected not in num_cube and cube_detected != 10):
            num_cube.append(cube_detected)
            #print("id",msg.markers[i].id,"/10",cube_detected)
            print("cubes presentes",num_cube)
            
        
    liste_cubes=  ListeCubes()
    for i in range(0,len(num_cube)):
        cube = Cube()
        cube.id = num_cube[i]

        
        
        liste_cubes.cubes.append(cube)
    

rospy.init_node('publisher_camera')   #initialisation du noeud et affectation d'un nom


# definition de trois publishers et d'un subscriber

pub_cube = rospy.Publisher('/cube_presents', ListeCubes, queue_size=10)     
pub_robot = rospy.Publisher('/braccio_arm/joint_angles', Float64MultiArray, queue_size=10)     
pub_camera = rospy.Publisher('/braccio_arm/camera_joint_angles', Float64MultiArray, queue_size=10) 
rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, callbackAruco)
                                                        
rate = rospy.Rate(10)                       #definit la frequence d envoie 10Hz

RotZ_camera = 0
TranX_camera = 0.8
TranZ_camera = 0.5
RotY_camera = 0.5

msg_robot1.data = [RotZ_camera,TranX_camera,TranZ_camera,RotY_camera]

pub_camera.publish(msg_camera1)

while not rospy.is_shutdown():         #tant que ros est en fonctionnement  
    
    if RotZ_camera < 2*math.pi:
        msg_camera1.data = [RotZ_camera,TranX_camera,TranZ_camera,RotY_camera]
        RotZ_camera = RotZ_camera+1
        pub_camera.publish(msg_camera1)
        rospy.sleep(2.)

    
    pub_cube.publish(liste_cubes)
    
    rate.sleep()                       # a remettre apres exemple
    
