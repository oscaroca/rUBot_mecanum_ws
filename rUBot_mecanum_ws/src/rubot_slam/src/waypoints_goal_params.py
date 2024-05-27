#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees, radians
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

def create_initpose(position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, orientation_z)
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = 'map'
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.pose.position.x = position_x
    pose_msg.pose.pose.position.y = position_y
    pose_msg.pose.pose.position.z = 0.0
    pose_msg.pose.pose.orientation.x = q_x
    pose_msg.pose.pose.orientation.y = q_y
    pose_msg.pose.pose.orientation.z = q_z
    pose_msg.pose.pose.orientation.w = q_w
    return pose_msg

def create_pose_stamped(position_x, position_y, rotation_z):
    goal = MoveBaseGoal()# Has to be created here
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rotation_z)
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = position_x
    goal.target_pose.pose.position.y = position_y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = q_x
    goal.target_pose.pose.orientation.y = q_y
    goal.target_pose.pose.orientation.z = q_z
    goal.target_pose.pose.orientation.w = q_w
    return goal

def init_pose():
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    initial_pose = create_initpose(0.5, -0.5, radians(-90))
    rate = rospy.Rate(1) # 1hz has to be low value
    pub.publish(initial_pose)
    rate.sleep()
    rospy.loginfo("Init Pose done!")

def movebase_client():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goals = []
    i = 1
    while True:
        try:
            goal = rospy.get_param(f"~goal{i}")
            if goal is None:
                break
            goals.append(goal)
            i += 1
        except:
            break


    # Create pose for each goal
    waypoints = []
    for goal in goals:
        pose = create_pose_stamped(goal['x'], goal['y'], radians(goal['w']))
        waypoints.append(pose)

    # --- Follow Waypoints ---

    for i in range(len(waypoints)):
        client.send_goal(waypoints[i])
        finished = client.wait_for_result(rospy.Duration(20))# timeout


        if finished:
            state = client.get_state()
            status_text = client.get_goal_status_text() #depende el status esperar para volver a reorganizar
            rospy.loginfo(status_text)
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation successful")
            else:
                rospy.logwarn("Navigation failed. State: %s", state)
                if i == len(waypoints)-1: #si es el ultimo hace shutdown, sino intenta ir al siguiente punto
                    rospy.logerr("Action server not available!")
                    rospy.signal_shutdown("Action server not available!")
        else:
            rospy.logwarn("Timeout reached. Navigation failed.")
            if i == len(waypoints)-1: #si es el ultimo hace shutdown, sino intenta ir al siguiente punto
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_waypoints')
        #init_pose()
        movebase_client()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
