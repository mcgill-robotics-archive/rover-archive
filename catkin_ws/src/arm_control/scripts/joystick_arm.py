#!/usr/bin/env python

# Script to create a node that subscribes to the "closed_arm" topic to pipe joystick Pose data into MoveIt

# Import statements
import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import std_msgs.msg
from std_msgs.msg import Bool

global group

# If closed_arm_bool's data is 'True' then we will execute the pose_target
def callback_Exec(data):
    # Instantiate a RobotCommander object which is an interface to the robot as a whole. Instantiate a PlanningSceneInterface object which is an interface to the world surrounding the robot.
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("Arm")

    if ( data.data ):
        rospy.loginfo("I'm TRUE`")
        # Instruct MoveIt to execute the pose_target
        group.go(wait=False)

# This will update the set_target with the Pose data that "closed_arm" sends
def callback_Pose(data):
    # Instantiate a RobotCommander object which is an interface to the robot as a whole. Instantiate a PlanningSceneInterface object which is an interface to the world surrounding the robot.
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("Arm")

    rospy.loginfo("I've set the POSE")
    # Set the pose_target by passing it the Pose message we receive from our topic
    group.set_pose_target(data)


def main():
    rospy.init_node('joystick_data', anonymous=True)



    # Create the subscribers on this node that, together, will execute Pose targets for the arm
    rospy.Subscriber("closed_arm_pose", Pose, callback_Pose)
    rospy.Subscriber("closed_arm_bool", Bool, callback_Exec)
    rospy.spin()

if __name__ == '__main__':
    main()



