#!/usr/bin/env python3
import rospy
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees, radians
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import cv2 as cv
from rubot_project1_picture import TakePhoto
from TrafficSignalsDetection_sp import combined_detection, KerasImageClassifier, signal_detected_fast

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
   
def nav2goals():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
    client.wait_for_server()
    
    goal_s = rospy.get_param("~goal_s")
    goal_r = rospy.get_param("~goal_r")
    goal_l = rospy.get_param("~goal_l")
    goal_t = rospy.get_param("~goal_t")
    img_topic = rospy.get_param("~img_topic")

    goal_pose_s = create_pose_stamped(goal_s['x'], goal_s['y'], radians(goal_s['w']))
    goal_pose_r = create_pose_stamped(goal_r['x'], goal_r['y'], radians(goal_r['w']))
    goal_pose_l = create_pose_stamped(goal_l['x'], goal_l['y'], radians(goal_l['w']))
    goal_pose_t = create_pose_stamped(goal_t['x'], goal_t['y'], radians(goal_t['w']))

    name_photo_s= photos_path + goal_s['photo_name']
    
    classifier = KerasImageClassifier(rubot_projects_path+ "/src/va/4 classes/keras_model.h5", rubot_projects_path + "/src/va/4 classes/labels.txt") #Poner el path absoluto
    

    # First goal and Take photo signal right
    client.send_goal(goal_pose_s)
    wait = client.wait_for_result(rospy.Duration(30))
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal execution done!") 
        camera = TakePhoto(img_topic, name_photo_s)
        # Important! Allow up to one second for connection
        rospy.sleep(1)
        #camera.save_picture(name_photo_s)

    # Process photo signal
    traffic_signal = combined_detection(name_photo_s, classifier)[:-1]
    if traffic_signal == "Derecha":
        rospy.loginfo("Signal detected: Right!")
        waypoints = [goal_pose_r, goal_pose_t]
    elif traffic_signal == "Izquierda":
        rospy.loginfo("Signal detected: Left!")
        waypoints = [goal_pose_l, goal_pose_t]
    elif traffic_signal == "Stop":
        rospy.loginfo("Signal detection: STOP!, esperant 20 seg")
        rospy.sleep(20)
        waypoints = [goal_pose_l, goal_pose_t]
    elif traffic_signal == "Ceda":
        rospy.loginfo("Signal detection: CEDA esperant 10 seg")
        rospy.sleep(10)
        waypoints = [goal_pose_l, goal_pose_t]
    else:
        rospy.loginfo("Signal detected: Right!")
        waypoints = [goal_pose_r, goal_pose_t]
    # --- Follow Waypoints ---
    for i in range(2):
        client.send_goal(waypoints[i])
        wait = client.wait_for_result(rospy.Duration(30))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal execution done!")
            if i == 1: 
                camera = TakePhoto(img_topic, name_photo_s)
                # Important! Allow up to one second for connection
                rospy.sleep(1)
                traffic_signal = combined_detection(name_photo_s, classifier)[:-1]
                rospy.loginfo("Signal detected: Left!")

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_waypoints')
        # Initialize the ROS package manager
        rospack = rospkg.RosPack()
        # Get the path of the 'rubot_projects' package
        rubot_projects_path = rospack.get_path('rubot_projects')
        # Construct the full path to the photos directory
        photos_path = rubot_projects_path + '/photos/'
        nav2goals()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")