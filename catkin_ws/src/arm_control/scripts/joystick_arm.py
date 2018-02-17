#!/usr/bin/env python

# Script to create a node that subscribes to the "closed_arm" topic to pipe joystick Pose data into MoveIt

# Import statements
import rospy
import moveit_commander
import geometry_msgs.msg
import std_msgs.msg

# If closed_arm_bool's data is 'True' then we will execute the pose_target
def callback_Exec(data):
    if ( data.data ):
        # Instruct MoveIt to execute the pose_target
        group.go(wait=False)

# This will update the set_target with the Pose data that "closed_arm" sends
def callback_Pose(data):
    # Set the pose_target by passing it the Pose message we receive from our topic
    group.set_pose_target(data.data)


def main():
    rospy.init_node('joystick_data', anonymous=True)

    # Instantiate a RobotCommander object which is an interface to the robot as a whole. Instantiate a PlanningSceneInterface object which is an interface to the world surrounding the robot.
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("Arm")

    # Create the subscribers on this node that, together, will execute Pose targets for the arm
    rospy.Subscriber("closed_arm_pose", Pose, callback_Pose)
    rospy.Subscriber("closed_arm_bool", Bool, callback_Exec)
    rospy.spin()

if __name__ == '__main__':
    main()



