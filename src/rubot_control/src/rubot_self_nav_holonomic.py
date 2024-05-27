#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rubot_nav import move_rubot
from math import sqrt,sin,cos,radians


#Interesa trabajar con angulos positivos y negativos (mano derecha regla). 
#De este modo podemos trabajr con la omega mejor, si positivo podemos cambiar la omega a negativo cuando nos movemos

class rUBot:

    def __init__(self):

        rospy.init_node("rubot_nav", anonymous=False)
        self._distanceLaser = rospy.get_param("~distance_laser") #Distancia mínima
        self._speedFactor = rospy.get_param("~speed_factor")# para multiplicar todas las velocidades. A 0 se utiliza para debugg
        self._forwardSpeed = rospy.get_param("~forward_speed")# Velocidad por defecto hacia adelante (0.2 es bastante)
        self._backwardSpeed = rospy.get_param("~backward_speed")# si distancia minima usar esta velocidad para regular
        self._rotationSpeed = rospy.get_param("~rotation_speed")# Si se rota usar esta velocida, usar un signo

        self._msg = Twist()
        self._cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callbackLaser)
        rospy.on_shutdown(self.shutdown)

        self._r = rospy.Rate(1)
        self._previousmsg=Twist()
        self._previousmsg.linear.x = self._forwardSpeed * self._speedFactor
        self._previousmsg.angular.z = 0
        
        # Propiedades secundarias

        # Our Lidar has more than 720 laser beams and not all the Lidars have the same number
        # Se debe de calcular en la primera ejecucion de __callbackLaser(). Esta
        # variable sirve para asegurar que solo se ejecuta este calculo del
        # factor de correccion una sola vez.
        #self.__isScanRangesLengthCorrectionFactorCalculated = False
        #self.__scanRangesLengthCorrectionFactor = 2

    def start(self):

        while not rospy.is_shutdown():
            self._cmdVel.publish(self._msg)
            self._r.sleep()

    def callbackLaser(self, scan):
        """Funcion ejecutada cada vez que se recibe un mensaje en /scan."""
        # En la primera ejecucion, calculamos el factor de correcion del Lidar
                
        closestDistance, elementIndex = min((val, idx) for (idx, val) in enumerate(scan.ranges) if scan.range_min < val < scan.range_max) #Cogemos la distancia mínima
        #range_max es para evitar los inf
        angleClosestDistance = (elementIndex / 2)  #ALERTAAAAA SE DIVIDE ENTRE 2 por que en simulación hay 720 ejes en el lindar (lenght del array de ranges)

        angleClosestDistance= self.__wrapAngle(angleClosestDistance)
        rospy.loginfo("Degree wraped %5.2f ",(angleClosestDistance))
        closestDistance

        if angleClosestDistance > 0: #El 0 del lidar esta detrás, esto hace que apunte delante y que el angulo 0 sea de frente
            angleClosestDistance=(angleClosestDistance-180)
        else:
            angleClosestDistance=(angleClosestDistance+180)
			
        rospy.loginfo("Closest distance of %5.2f m at %5.1f degrees.",closestDistance, angleClosestDistance)

        
        if False: #Algoritmo base  

            if closestDistance < self._distanceLaser and -80 < angleClosestDistance < 80:
                self._msg.linear.x = self._backwardSpeed * self._speedFactor
                #Signo contrario a closest distance para alejarse, y rotation speed
                self._msg.angular.z = -self.__sign(angleClosestDistance) * self._rotationSpeed * self._speedFactor
                rospy.logwarn("Within laser distance threshold. Rotating the robot (z=%4.1f)...", self._msg.angular.z)

            else:
                #Sino goes para delante
                self._msg.linear.x = self._forwardSpeed * self._speedFactor
                self._msg.angular.z = 0

        elif True: #A pesar de que el robot es holonomico....
        
            if closestDistance < self._distanceLaser:
                if -30 <= angleClosestDistance <= 30:
                    print("frontal")
                    self._msg.linear.x = self._forwardSpeed * self._speedFactor
                    self._msg.angular.z = self.__sign(angleClosestDistance+90) * self._rotationSpeed * self._speedFactor
                    
                elif 30 < angleClosestDistance <= 150:
                    print("izquierda")
                    self._msg.linear.x = self._forwardSpeed * self._speedFactor
                    new_angle=angleClosestDistance+90
                    if new_angle> 180:
                        new_angle=-360 + new_angle
                        self._msg.angular.z = self.__sign(new_angle) * self._rotationSpeed * self._speedFactor
                    else:
                        self._msg.angular.z = self.__sign(new_angle) * self._rotationSpeed * self._speedFactor
                
                elif -150 <= angleClosestDistance < -30:
                    print("derecha")
                    self._msg.linear.x = self._forwardSpeed * self._speedFactor
                    self._msg.angular.z = self.__sign(angleClosestDistance+90) * self._rotationSpeed * self._speedFactor
                
                else:# 150 - -150
                    print("trasera")
                    self._msg.linear.x = self._forwardSpeed * self._speedFactor
                    new_angle=angleClosestDistance+90
                    if new_angle> 180:
                        new_angle=-360 + new_angle
                        self._msg.angular.z = self.__sign(new_angle) * self._rotationSpeed * self._speedFactor
                    else:
                        self._msg.angular.z = self.__sign(new_angle) * self._rotationSpeed * self._speedFactor
            else:
                self._msg.linear.x = self._forwardSpeed * self._speedFactor
                self._msg.angular.z = 0

        else:#Full Holonomico

            linear_x, linear_y = self.calculate_velocities(closestDistance, angleClosestDistance)
            self._msg.linear.x = linear_x
            self._msg.linear.y = linear_y
               


    def calculate_velocities(self, closestDistance, angleClosestDistance):
        linear_x = self._forwardSpeed * self._speedFactor
        linear_y = 0

        if closestDistance < self._distanceLaser:
            new_angle=angleClosestDistance + 90
            print(new_angle)
            if new_angle> 180:
                new_angle=-360 + new_angle
            print(new_angle)
            angle_rad = radians(new_angle)  # Convert to radians
            linear_x = linear_x * cos(angle_rad)
            
            linear_y = linear_x * sin(angle_rad)
            if new_angle>90 or new_angle<-90:
                linear_y=linear_y*-1
            print("Velocity vector:", linear_x, linear_y)

        return linear_x, linear_y


    def __sign(self, val):

        if val >= 0:
            return 1
        else:
            return -1

    def __wrapAngle(self, angle):
        if 0 <= angle <= 180:
            return angle
        else:
            return angle - 360

    def shutdown(self):
        self._msg.linear.x = 0
        self._msg.linear.y = 0
        self._msg.angular.z = 0
        self._cmdVel.publish(self._msg)
        rospy.loginfo("Stop RVIZ")

        import math

    def laberinto_solver(self,angulo_original):
       
       #Siempre se movera en angulo recto a la pared
        angulo_radianes = self.__wrapAngle(angulo_original + 90)
        

        # Calcular los componentes x e y de la velocidad
        velocidad_x = cos(angulo_radianes)
        velocidad_y = sin(angulo_radianes)

        return velocidad_x,velocidad_y


if __name__ == '__main__':
    try:
        rUBot1 = rUBot()
        rUBot1.start()
        rospy.spin()
        rUBot1.shutdown()
    except rospy.ROSInterruptException: rUBot1.shutdown()#pass
