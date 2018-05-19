#!/usr/bin/env python

# Script to create a node that subscribes to the "closed_arm" topics (bool and posititon) to pipe joystick Pose data into MoveIt

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
        rospy.loginfo("I'm executing the plan!")
        group.execute(plan1)



# This will update the set_target with the Pose data that "closed_arm_pose" sends
def callback_Pose(data):

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



