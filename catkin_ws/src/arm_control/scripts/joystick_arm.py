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
    # Instantiate a RobotCommander object which is an interface to the robot as a whole
    # Instantiate a PlanningSceneInterface object which is an interface to the world surrounding the robot.

    '''
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("Arm")
    
    if ( data.data == True ):
        rospy.loginfo("I'm TRUE !!")
        #group.execute(wait=False)
        group.go(wait=False)
    '''

    if ( data.data == True ):
        # Instantiate a RobotCommander object which is an interface to the robot as a whole & a PlanningSceneInterface object which is an interface to the world surrounding the robot.
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("Arm")

        # Our pose target for debugging
        pose_target = geometry_msgs.msg.Pose()
        
        # Position fields
        pose_target.position.x = -0.29304
        pose_target.position.y = -0.59885
        pose_target.position.z = 0.48658

        # Orientation fields
        pose_target.orientation.x = 0.73143
        pose_target.orientation.y = 0.26853
        pose_target.orientation.z = 0.52608
        pose_target.orientation.w = -0.34082

        # Set the Pose target and plan it
        group.set_pose_target(pose_target)
        plan1 = group.plan()
        group.execute(plan1)

    '''
    if ( data.data ):
        rospy.loginfo("I'm TRUE`")
        # Instruct MoveIt to execute the pose_target
        group.go(wait=False)
    '''

    #rospy.loginfo("I'm TRUE")
    #group.go(wait=False)

# This will update the set_target with the Pose data that "closed_arm_pose" sends
def callback_Pose(data):


    '''
    # Set the pose_target by passing it the Pose message we receive from our topic
    group.set_pose_target(data)
    '''   

    '''
    # Instantiate a RobotCommander object which is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()
    # Instantiate a PlanningSceneInterface object which is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("Arm")
    group.set_pose_target(data)
    group.plan()
    '''    

    rospy.loginfo("I've set the POSE")

    # This will print out the x value of the Point field of the Pose msg
    # rospy.loginfo("X data is: %s", data.position.x)

def main():
    rospy.init_node('joystick_data', anonymous = True)

    # Create the subscribers on this node that, together, will execute Pose targets for the arm
    rospy.Subscriber("closed_arm_pose", Pose, callback_Pose)
    rospy.Subscriber("closed_arm_bool", Bool, callback_Exec)
    rospy.spin()

if __name__ == '__main__':
    main()



