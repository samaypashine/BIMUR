#!/usr/bin/env python

import sys
import os
import socket
import json
import wave

import rospy
import rospkg
from joint_recorder.msg import recorderMsg
from audio_common_msgs.msg import AudioData


def audio_callback(msg):

	global folderName, recordFlag, audio_msgs
	if recordFlag:
		audio_msgs.append(msg.data)


def callback(msg):
	# print the actual message in its raw format
	# rospy.loginfo("Here's what was subscribed:", msg.fileName.data, msg.command.data, msg.topic.data)
	
	# otherwise simply print a convenient message on the terminal
	# print('Data from /recording_control_topic received', msg.fileName.data, msg.command.data, msg.topic.data)

	global folderName, recordFlag, audio_msgs
	if msg.command.data == "set_file_name" and msg.topic.data == thisNodeName:
		rospy.loginfo("Setted the File name.")
		folderName = msg.fileName.data
	elif folderName:
		if msg.command.data == "start":
			rospy.loginfo("In Start condition of recordService callback function")
			recordFlag = True
		elif msg.command.data == "stop":
			rospy.loginfo("In STOP condition of recordService callback function")

			channels = 1
			rate = 16000  # 16000, 44100

			wf = wave.open(folderName, 'wb')
			wf.setnchannels(channels)
			wf.setsampwidth(2L)
			wf.setframerate(rate)
			wf.writeframes(b''.join(audio_msgs))
			wf.close()

			folderName = ""
			recordFlag = False
			audio_msgs = []
		elif msg.command.data == "shutdown":
			rospy.logerr("Shutting down")
			rospy.on_shutdown()
		else:
			rospy.logerr("Command Should be \"set_file_name\" \"start\" or \"stop\" or or \"shutdown\"")


def main():
	# initialize a node by the name 'listener'.
	# you may choose to name it however you like,
	# since you don't have to use it ahead
	
	rospy.init_node('audio_recorder_node', anonymous=True)
	rospy.Subscriber("recording_control_topic", recorderMsg, callback)
	rospy.Subscriber("/audio", AudioData, audio_callback)
	
	# spin() simply keeps python from
	# exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	
	thisNodeName = "audio_capture"
	folderName = ""
	recordFlag = False
	audio_msgs = []

	# you could name this function
	try:
		main()
	except rospy.ROSInterruptException:
		pass
