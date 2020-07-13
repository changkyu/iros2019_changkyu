#!/usr/bin/env python
'''
File: SModelPublisherController.py

Authors: Aravind Sivaramakrishnan

Description: Instead of getting raw input from the user (like SModelSimpleController.py does), this script
constantly listens to a topic and receives the command from there.

Comments/TODO:
- Too many flags, I am going mad
'''
import roslib; roslib.load_manifest('robotiq_s_model_control')
import rospy
import sys
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg
from SModelSimpleController import genCommand
from std_msgs.msg import String, Bool
from time import sleep

newCommandAvailable = False

def cb(data):
	global command_msg, newCommandAvailable
	newCommandAvailable = True
	command_msg = data.data
	print "\n\n\nObtained gripper message ",command_msg,"\n\n\n"


def force_cb(status):
	global exceeded_force, force_pub
	# if (status.gDTA == 2 or status.gDTB == 2 or status.gDTC == 2):
	# 	exceeded_force = True
	# 	# print "FORCE EXCEEDED ON FINGER..."
	# 	force_pub.publish('true')
	# 	# print "Publishing to force_pub true " 
	# print "GG[",status.gDTA,status.gDTB,status.gDTC,"]"
	force_count = 0
	if (status.gDTA == 2):
		force_count = force_count + 1
	if (status.gDTB == 2):
		force_count = force_count + 1
	if (status.gDTC == 2):
		force_count = force_count + 1
	if force_count >=2:
		exceeded_force = True
		force_pub.publish('true')
		# print "Publishing to force_pub true " 
	else:
		exceeded_force = False
		force_pub.publish('false')
		# print "Publishing to force_pub false" 

# def publisher(keep_gripper_status):
# 	global command_msg, newCommandAvailable
# 	"""Main loop which subscribes to new commands and publish them on the SModelRobotOutput topic."""
# 	rospy.init_node('SModelPublisherController')

# 	command_pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size=10)
# 	command_force_sub = rospy.Subscriber('SModelRobotInput', inputMsg.SModel_robot_input, force_cb)
# 	command_sub = rospy.Subscriber('GripperCommand', String, cb)

# 	command = outputMsg.SModel_robot_output()

# 	firstTime = True

# 	newMessageAvailable = False

# 	while not rospy.is_shutdown():
# 		if firstTime:
# 			# Activate the gripper.
# 			if keep_gripper_status == 'n':
# 				rospy.loginfo("Reactivating the gripper...")
# 				command = genCommand('a',command)
# 				newMessageAvailable = True
# 			firstTime = False 
# 		else:
# 			if newCommandAvailable:
# 				command = genCommand(command_msg,command)
# 				newCommandAvailable = False
# 				newMessageAvailable = True

# 		if newMessageAvailable and command_pub.get_num_connections() > 0:
# 			command_pub.publish(command)
# 			newMessageAvailable = False

# 		rospy.sleep(0.1)

# 	rospy.spin()

def publisher(keep_gripper_status):
	global command_msg, newCommandAvailable, force_pub
	"""Main loop which subscribes to new commands and publish them on the SModelRobotOutput topic."""
	rospy.init_node('SModelPublisherController')

	command_pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size=10)
	command_force_sub = rospy.Subscriber('SModelRobotInput', inputMsg.SModel_robot_input, force_cb)
	command_sub = rospy.Subscriber('GripperCommand', String, cb)
	force_pub = rospy.Publisher('/GripperForceFeedback', String, queue_size=10)

	command = outputMsg.SModel_robot_output()

	firstTime = True

	newMessageAvailable = False
	sent_command = False

	while not rospy.is_shutdown():
		if firstTime:
			# Activate the gripper.
			if keep_gripper_status == 'n':
				rospy.loginfo("Reactivating the gripper...")
				command = genCommand('a',command)
				newMessageAvailable = True
			firstTime = False 
		else:
			if newCommandAvailable:
				try:
					command_int = int(command_msg)
					if(command_int<120):
						exit_cause = True
						while(exit_cause):
							if command_pub.get_num_connections() > 0:
								command_pub.publish('i')
								newMessageAvailable = False
								rospy.sleep(0.1)
							exit_cause = (not exceeded_force)
						sent_command = True
				except:
					pass

				if not sent_command:
					command = genCommand(command_msg,command)
				
				newCommandAvailable = False
				newMessageAvailable = True

		if not sent_command and newMessageAvailable and command_pub.get_num_connections() > 0:
			command_pub.publish(command)
			newMessageAvailable = False

		rospy.sleep(0.1)

	rospy.spin()

if __name__ == '__main__':
	keep_gripper_status = rospy.myargv(sys.argv)[1]
	publisher(keep_gripper_status)	