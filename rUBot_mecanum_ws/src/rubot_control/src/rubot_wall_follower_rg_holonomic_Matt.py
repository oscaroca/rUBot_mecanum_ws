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
prev_move = 0
opt_pose = 0
isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 2


def clbk_laser(msg):
    # En la primera ejecucion, calculamos el factor de correcion
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    
    if not isScanRangesLengthCorrectionFactorCalculated:
            scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
            isScanRangesLengthCorrectionFactorCalculated = True

    bright_min = int(30 * scanRangesLengthCorrectionFactor)
    bright_max = int(90 * scanRangesLengthCorrectionFactor)
    right_min = int(90 * scanRangesLengthCorrectionFactor)
    right_max = int(120 * scanRangesLengthCorrectionFactor)
    fright_min = int(120 * scanRangesLengthCorrectionFactor)
    fright_max = int(170 * scanRangesLengthCorrectionFactor)
    front_min= int(170 * scanRangesLengthCorrectionFactor)
    front_max = int(190 * scanRangesLengthCorrectionFactor)
    fleft_min = int(190 * scanRangesLengthCorrectionFactor)
    fleft_max = int(240 * scanRangesLengthCorrectionFactor)
    left_min = int(240 * scanRangesLengthCorrectionFactor)
    left_max = int(270 * scanRangesLengthCorrectionFactor)
    bleft_min = int(270 * scanRangesLengthCorrectionFactor)
    bleft_max = int(330 * scanRangesLengthCorrectionFactor)
    back_min = int(330 * scanRangesLengthCorrectionFactor)
    back_max = int(360 * scanRangesLengthCorrectionFactor)


    regions = {
        'bright':  min(min(msg.ranges[bright_min:bright_max]), 3),
        'right':  min(min(msg.ranges[right_min:right_max]), 3),
        'fright': min(min(msg.ranges[fright_min:fright_max]), 3),
        'front':  min(min(msg.ranges[front_min:front_max]), 3),
        'fleft':  min(min(msg.ranges[fleft_min:fleft_max]), 3),
        'left':  min(min(msg.ranges[left_min:left_max]), 3),
        'bleft':  min(min(msg.ranges[bleft_min:bleft_max]), 3),
        'back':  min(min(msg.ranges[back_min:back_max]), 3),
    }

    take_action(regions)


def take_action(regions):
    global prev_move
    global opt_pose
    msg = Twist()
    linear_x = 0
    linear_y = 0
    angular_z = 0

    state_description = ''

    if (prev_move == 0):
        state_description = 'Sigue buscando, sigue buscando... (Nemo Reference)'
        linear_x = vx
        linear_y = 0
        if (regions['front'] < d or regions['fright'] < d or regions['right'] < d or regions['bright'] < d or regions['back'] < d or regions['bleft'] < d or regions['left'] < d or regions['fleft'] < d):
            prev_move=99

    elif (opt_pose == 0):
        state_description = 'Rotando'
        linear_x = 0
        linear_y = 0
        angular_z = vx
        if (regions['front'] > d and regions['right'] < d and regions['fright'] > d):
            state_description = 'Rotacion Optima'
            opt_pose = 99
            angular_z = 0

    elif regions['front'] > d and regions['right'] < d:
        state_description = 'Pared a la derecha: Sigo recto'
        linear_x = vx/1.5
        linear_y = 0
    
    elif regions['front'] < d and regions['left'] > d:
        state_description = 'Pared enfrente: Voy hacia la izquierda'
        linear_x = 0
        linear_y = vx/1.5
    
    elif regions['back'] > d and regions['left'] < d  :
        state_description = 'Pared a la izquierda: Voy para detras'
        linear_x = -vx
        linear_y = 0
    
    elif regions['back'] < d:
        state_description = 'Pared detras: Voy para la derecha'
        linear_x = 0
        linear_y = -vx/1.5

    elif regions['fleft'] < d*2:
        state_description = 'Pared a la izquierda-arriba: Voy para la izquierda'
        linear_x = vx/3
        linear_y = vx/1.5
    
    elif regions['fright'] < d*2:
        state_description = 'Pared a la derecha-arriba: Voy para adelante'
        linear_x = vx/1.5
        linear_y = -vx/3
        
    elif regions['bleft'] < d*2:
        state_description = 'Pared a la derecha-abajo: Voy para detras'
        linear_x = -vx/1.5
        linear_y = 0

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

    rospy.init_node('wall_follower')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(25)

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
