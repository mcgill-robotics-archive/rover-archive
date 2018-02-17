#!/usr/bin/env python

# Script to create a node that subscribes to the "closed_arm" topic to pipe joystick Pose data into MoveIt

# Import statements
import rospy
import moveit_commander
import geometry_msgs.msg


def callback(data):
    # Set the pose_target by passing it the Pose message we receive from our topic
    group.set_pose_target(data.data)
    # Instruct MoveIt to execute the pose_target
    group.go(wait=False)

def listener():
    rospy.init_node('joystick_listener', anonymous=True)

    # Instantiate a RobotCommander object which is an interface to the robot as a whole. Instantiate a PlanningSceneInterface object which is an interface to the world surrounding the robot.
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    # Create a subscriber on this node
    rospy.Subscriber("closed_arm", Pose, callback)
    rospy.spin()

if __name__=='__main__':
    listener()



