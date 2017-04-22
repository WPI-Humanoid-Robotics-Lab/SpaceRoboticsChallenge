#!/usr/bin/env python

import copy
import time
import rospy

from numpy import append

from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from ihmc_msgs.msg import HandDesiredConfigurationRosMessage
from ihmc_msgs.msg import HandTrajectoryRosMessage
from ihmc_msgs.msg import SE3TrajectoryPointRosMessage

ZERO_VECTOR = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ELBOW_BENT_UP = [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0]
BUTTON_PRESS = [0.0, 0.1, 0.2, 0.3, 1.5, -0.5, -0.20]

ROBOT_NAME = None

def sendRightArmTrajectory():
    msg = ArmTrajectoryRosMessage()

    msg.robot_side = ArmTrajectoryRosMessage.RIGHT
    position = [0.00305, -0.0911, 0.6201, 0.003, -0.008,-0.0016, 0.0]
    msg = appendTrajectoryPoint(msg, 2.0, position)
#    msg = appendTrajectoryPoint(msg, 3.0, ELBOW_BENT_UP)
#    msg = appendTrajectoryPoint(msg, 1.0, ZERO_VECTOR)
#    msg = appendTrajectoryPoint(msg, 4.0, ZERO_VECTOR)

#    msg = appendTrajectoryPoint(msg, 2.0, BUTTON_PRESS)
    msg.unique_id = -1

    rospy.loginfo('publishing right trajectory')
    armTrajectoryPublisher.publish(msg)
    time.sleep(0.5)
    msg.robot_side = ArmTrajectoryRosMessage.LEFT
    armTrajectoryPublisher.publish(msg)

def closeHand():
    msg = HandDesiredConfigurationRosMessage()
    msg.robot_side = HandDesiredConfigurationRosMessage.RIGHT
    msg.hand_desired_configuration = HandDesiredConfigurationRosMessage.CLOSE

    msg.unique_id = 2
    rospy.loginfo('Closing right hand')
    handTrajectoryPublisher.publish(msg)


def appendTrajectoryPoint(arm_trajectory, time, positions):
    if not arm_trajectory.joint_trajectory_messages:
        arm_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
        point = TrajectoryPoint1DRosMessage()
        point.time = time
        point.position = pos
        point.velocity = 0
        arm_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return arm_trajectory

def sendTaskSpaceTrajectory():
    msg = HandTrajectoryRosMessage()
    
    msg.robot_side = HandTrajectoryRosMessage.RIGHT
    msg.base_for_control = HandTrajectoryRosMessage.CHEST
        
    pt = SE3TrajectoryPointRosMessage()
    pt.time = 3.0

    #Home
    pt.position.x = 0.152
    pt.position.y = -0.754
    pt.position.z = 1.353
    pt.orientation.x = -0.045
    pt.orientation.y = 0.232
    pt.orientation.z = 0.045
    pt.orientation.w = 0.943
#    pt.orientation.x = 0.535
#    pt.orientation.y = -0.147
#    pt.orientation.z = 0.740
#    pt.orientation.w = 0.381

#    #Initial Manip
#    pt.position.x = 0.533
#    pt.position.y = -0.206
#    pt.position.z = 1.019
#    pt.orientation.x = -0.292
#    pt.orientation.y = -0.189
#    pt.orientation.z = 0.779
#    pt.orientation.w = 0.565

#    #Towards the handle
#    pt.position.x = 2.383
#    pt.position.y = 0.850
#    pt.position.z = 0.830
#    pt.orientation.x = -0.168
#    pt.orientation.y = -0.230
#    pt.orientation.z = 0.958
#    pt.orientation.w = 0.014

#    #grabing
#    pt.position.x = 2.253
#    pt.position.y = 0.875
#    pt.position.z = 0.800
#    pt.orientation.x = -0.168
#    pt.orientation.y = -0.230
#    pt.orientation.z = 0.958
#    pt.orientation.w = 0.014

#    Moving
#    pt.position.x = 1.850
#    pt.position.y = 0.990
#    pt.position.z = 0.500
#    pt.orientation.x = -0.168
#    pt.orientation.y = -0.230
#    pt.orientation.z = 0.958
#    pt.orientation.w = 0.014



    pt.unique_id = 255
    
    msg.taskspace_trajectory_points.append(pt)
    
    msg.execution_mode = HandTrajectoryRosMessage.OVERRIDE
    msg.unique_id = 212
    rospy.loginfo('Moving Hand in task space')
    taskSpaceTrajectoryPublisher.publish(msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_arm_demo1')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
        print ROBOT_NAME

        armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_trajectory".format(ROBOT_NAME), ArmTrajectoryRosMessage, queue_size=1)
        handTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/hand_desired_configuration".format(ROBOT_NAME), HandDesiredConfigurationRosMessage, queue_size=1)
        
        taskSpaceTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/hand_trajectory".format(ROBOT_NAME), HandTrajectoryRosMessage, queue_size=1)
        
        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if armTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subscriber...')
            while armTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendRightArmTrajectory()
            #closeHand()
            #sendTaskSpaceTrajectory()
            time.sleep(2)

    except rospy.ROSInterruptException:
        pass
