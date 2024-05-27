#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

pub = None
d = 0
vx = 0
wz = 0
vf = 0

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 3.186111111
wall= False


def clbk_laser(msg): #callback para lidar
    # En la primera ejecucion, calculamos el factor de correcion
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    
    if not isScanRangesLengthCorrectionFactorCalculated: #calculas el factor de array laser - grados (lidar factor)
            scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
            isScanRangesLengthCorrectionFactorCalculated = True

    #Definiendo las zonas en grados. Con esto nos aseguramos que el front sea delante, el back detras, etc
    bright_min = int(10 * scanRangesLengthCorrectionFactor)
    bright_max = int(80 * scanRangesLengthCorrectionFactor)
    right_min = int(80 * scanRangesLengthCorrectionFactor)
    right_max = int(100 * scanRangesLengthCorrectionFactor)
    fright_min = int(100 * scanRangesLengthCorrectionFactor)
    fright_max = int(170 * scanRangesLengthCorrectionFactor)
    front_min= int(170 * scanRangesLengthCorrectionFactor)
    front_max = int(190 * scanRangesLengthCorrectionFactor)
    fleft_min = int(190 * scanRangesLengthCorrectionFactor)
    fleft_max = int(260 * scanRangesLengthCorrectionFactor)
    left_min = int(260 * scanRangesLengthCorrectionFactor)
    left_max = int(280 * scanRangesLengthCorrectionFactor)
    bleft_min = int(280 * scanRangesLengthCorrectionFactor)
    bleft_max = int(350 * scanRangesLengthCorrectionFactor)
    back_min = int(350 * scanRangesLengthCorrectionFactor)
    back_max = int(10 * scanRangesLengthCorrectionFactor)

    
    #defines la región, 
    regions = {
        'bright':  min(min(msg.ranges[bright_min:bright_max]), 3),
        'right':  min(min(msg.ranges[right_min:right_max]), 3),
        'fright': min(min(msg.ranges[fright_min:fright_max]), 3),
        'front':  min(min(msg.ranges[front_min:front_max]), 3),
        'left':   min(min(msg.ranges[left_min:left_max]), 3),
        'fleft':  min(min(msg.ranges[fleft_min:fleft_max]), 3),
        'back':   min(min(min(msg.ranges[back_min:int(359*scanRangesLengthCorrectionFactor)]),min(msg.ranges[0:back_max])), 3),
        'bleft':  min(min(msg.ranges[bleft_min:bleft_max]), 3),
    }
    
    take_action(regions)


#función para actuar
def take_action(regions):
    global wall
    msg = Twist()
    linear_x = 0
    linear_y = 0
    angular_z = 0

    #Print para ver los valores de las regiones
    print("Values")
    print("front :"+str(regions['front']))
    print("left :"+str(regions['left']))
    print("back :"+str(regions['back']))
    print("right :"+str(regions['right']))
    print("Distancia minima :"+str(d))
    state_description = ''

    #############################  Algoritmo  ############################


    #Si ha encontrado una pared
    if wall:


        #############################  Comentado: Si queremos mantener la orientacion, para que rote  ############################

        # min_region = min(regions.values())
        # if min_region=='fright': #obstaculo delante - derecha
        #     state_description = 'case 2 - fright'
        #     linear_x = 0
        #     angular_z = +wz * vf
        # elif min_region=='bright': #
        #     state_description = 'case 5 - bright'
        #     linear_x = 0
        #     angular_z = +wz* vf
        # elif min_region=='fleft': #obstaculo delante - izquierda
        #     state_description = 'case 6 - fleft'
        #     linear_x = 0
        #     angular_z = +wz* vf
        # elif min_region=='bleft': 
        #     state_description = 'case 8 - bleft'
        #     linear_x = 0
        #     angular_z = +wz* vf

        #Corners convexos
        
        if regions['front'] < d and regions['left'] < d: 
            state_description = 'case Mix - front-left'
            linear_x = -vx* vf
            linear_y= 0
            angular_z = 0
        elif regions['back'] < d and regions['left']< d: 
            state_description = 'case Mix - back-left'
            linear_x = 0
            linear_y= -vx * vf
            angular_z = 0
        elif regions['front'] < d and regions['right']< d: 
            state_description = 'case Mix - front-right'
            linear_x = 0
            linear_y= +vx * vf
            angular_z = 0
        elif regions['back'] < d and regions['right']< d: 
            state_description = 'case Mix - back-right'
            linear_x = +vx* vf
            linear_y= 0
            angular_z = 0
        
        #Paredes
            
        elif regions['front'] < d: # te encuentras un obstaculo delante - ir hacia la izquierda, ya que se supone que vas siguiendo la derecha
            state_description = 'case 2 - front'
            linear_x = 0
            linear_y= vx * vf
            angular_z = 0
        elif regions['right'] < d: # seguir con la pared a la derecha
            state_description = 'case 4 - right'
            linear_x = vx* vf
            angular_z = 0
        elif regions['left'] < d: # seguir con la pared a la izquierda
            state_description = 'case 7 - left'
            linear_x = -vx* vf
            angular_z = 0
        elif regions['back'] < d: #obstaculo detras
            state_description = 'case 9 - back'
            linear_x = 0
            linear_y= -vx * vf
            angular_z = 0

        #Para los corners exteriores concavos
        elif regions['fright'] < d: #obstaculo delante - derecha
            state_description = 'corner - fright'
            linear_x = +vx* vf
        elif regions['bright'] < d: #
            state_description = 'corner- bright'
            linear_y= - vx * vf
        elif regions['fleft'] < d: #obstaculo delante - izquierda
            state_description = 'corner - fleft'
            linear_y= vx * vf
        elif regions['bleft'] < d: 
            state_description = 'corner - bleft'
            linear_x = -vx* vf
        
        
        else: #no hay obstaculos - implica que hemos perdido la pared, esto solo puede pasar si se ha movido el entorno o los sensores no van bien
            state_description = 'case 10 - Far'
            linear_x = vx * vf
            angular_z = 0


    #SI no hemos encontrado pared, nos movemos hasta encontrarla
    else:
        if any(region < d for region in regions.values()):
            state_description = 'case 0 - WALL detected'
            wall = True
            linear_x = 0
            linear_y = 0
            angular_z = 0
        else:
            state_description = 'case 1 - No wall'
            linear_x = vx * vf
            linear_y = 0
            angular_z = 0


    
    #construimos el mensaje
    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    msg.angular.z = angular_z
    pub.publish(msg)
    rate.sleep()

def shutdown():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo("Stop rUBot")

def main():
    global pub
    global sub
    global rate
    global d
    global vx
    global wz
    global vf

    rospy.init_node('wall_follower') # nodo
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publisher y subscribers
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser) #callback
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(25) # Normalmente 10, pero incrementado para que hardware responda mejor

    #Leer parametros
    d= rospy.get_param("~distance_laser")
    vx= rospy.get_param("~forward_speed")
    wz= rospy.get_param("~rotation_speed")
    vf= rospy.get_param("~speed_factor")
    
    
if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        shutdown()

    


if __name__ == '__main__':
    main()
