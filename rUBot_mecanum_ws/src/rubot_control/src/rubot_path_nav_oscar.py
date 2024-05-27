#!/usr/bin/env python3
import rospy
from rubot_nav import move_rubot
from math import sqrt,sin,cos


def square_path(v,td):
    move_rubot(v,0,0,td)
    move_rubot(0,v,0,td)
    move_rubot(-v,0,0,td)
    move_rubot(0,-v,0,td)


def triangular_path(v, td):
    move_rubot(v,0,0,td)
    move_rubot(-v,v,0,td/sqrt(2))
    move_rubot(-v,-v,0,td/sqrt(2))

def diamond_path(v, td):
    move_rubot(v, 0, 0, td)
    print("STOPPP")
    move_rubot(0, v, 0, td)
    move_rubot(-v, 0, 0, td)
    move_rubot(0, -v, 0, td)
    move_rubot(v, 0, 0, td)

def circular_path(v, td):
    num_points = 20  # Number of points to approximate the circle
    r = v / (2 * sin(3.14 / num_points))
    for i in range(num_points):
        theta = i * 2 * 3.14 / num_points
        x = r * cos(theta)
        y = r * sin(theta)
        move_rubot(x, y, 0, td / num_points)

def linear_path(v,td):

    move_rubot(v,0,0,td)





if __name__ == '__main__':
    try:
        rospy.init_node('rubot_nav', anonymous=False)
        v= rospy.get_param("~v")
        w= rospy.get_param("~w")
        td= rospy.get_param("~td")
        path= rospy.get_param("~path")

        if path == "Square":
            square_path(v, td)
        elif path == "Triangular":
            triangular_path(v, td)
        elif path == "Diamond":
            diamond_path(v, td)
        elif path == "Circular":
            circular_path(v, td)
        elif path == "Linear":
            linear_path(v, td)
        else:
            print('Error: unknown movement')

    except rospy.ROSInterruptException:
        pass
