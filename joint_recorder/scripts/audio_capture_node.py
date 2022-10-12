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


# def reset_record_info(path):
# 	record_info = {"PATH": "", "RECORD": False}
# 	with open(path, "w") as outfile:
# 		json.dump(record_info, outfile)


def audio_callback(msg):
	global folderName, recordFlag, audio_msgs

	# rospy.loginfo(recordFlag)
	rospy.loginfo("folderName: " + folderName)
	rospy.loginfo(len(audio_msgs))
	rospy.loginfo(len(audio_msgs) > 0)
	# rospy.loginfo(msg)
	if recordFlag:
		audio_msgs.append(msg)
	else:
		print("hello")
	
	# if folderName and len(audio_msgs) > 0:
	if False:
		# rospy.loginfo(len(audio_msgs))
		# rospy.loginfo(len(audio_msgs) > 0)
		print("len(audio_msgs): ", len(audio_msgs))
		rospy.loginfo(len(audio_msgs))
		# channels = 2  # 1, 2
		rate = 44100  # 16000, 44100
        wf = wave.open(folderName, 'wb')
        wf.setnchannels(2)
        wf.setsampwidth(2L)
        wf.setframerate(rate)
        wf.writeframes(b''.join(audio_msgs))
        wf.close()

        folderName = ""
        audio_msgs = []


def callback(msg):
	
	# print the actual message in its raw format
	# rospy.loginfo("Here's what was subscribed:", msg.fileName.data, msg.command.data, msg.topic.data)
	
	# otherwise simply print a convenient message on the terminal
	# print('Data from /recording_control_topic received', msg.fileName.data, msg.command.data, msg.topic.data)

	global folderName, recordFlag, audio_msgs
	rospy.loginfo("Audio msg.command.data: " + msg.command.data)
	if msg.command.data == "set_file_name" and msg.topic.data == thisNodeName:
		rospy.loginfo("Setted the File name.")
		folderName = msg.fileName.data
		# sock.send(str.encode("PATH:" + folderName))
		# with open(record_info_path, 'r') as openfile:
		# 	record_info = json.load(openfile)
		# 	record_info["PATH"] = folderName
		# with open(record_info_path, "w") as outfile:
		# 	json.dump(record_info, outfile)
	elif folderName:
		if msg.command.data == "start":
			rospy.loginfo("In Start condition of recordService callback function")
			recordFlag = True
			rospy.loginfo("Audio start: " + str(recordFlag))

			# now = rospy.Time.now()
			# print("now: ", str(now.secs) + '.' + str(now.nsecs))

			# # sock.send(str.encode("RECORD:" + str(now.secs) + '.' + str(now.nsecs)))

			# with open(record_info_path, 'r') as openfile:
			# 	record_info = json.load(openfile)
			# 	record_info["RECORD"] = True
			# with open(record_info_path, "w") as outfile:
			# 	json.dump(record_info, outfile)

			# frame = digit.save_frame("/home/pc1/Downloads/Liquid_Dataset/touch_data/{}.jpg".format(args.num))

		elif msg.command.data == "stop":
			rospy.loginfo("In STOP condition of recordService callback function")

			

	        recordFlag = False
	        rospy.loginfo("Audio stop: " + str(recordFlag))

   		if msg.command.data == "shutdown":
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
	recordFlag = False
	audio_msgs = []
	folderName = ""

	# rospack = rospkg.RosPack()

	# digit_path = rospack.get_path('digit')
	# record_info_path = digit_path + os.sep + "src/record_info.json"
	# rospy.loginfo(record_info_path)

	# # record_info = {"PATH": "", "RECORD": False}
	# # 
	# # with open(record_info_path, "w") as outfile:
	# # 	json.dump(record_info, outfile)

	# reset_record_info(record_info_path)

	# sys.path.append(digit_path + "/src/digit-interface")
	# from digit_interface.digit import Digit

	# digit = Digit("D20501", "Left Gripper")
	# print("digit: ", digit.dev_name)
	# digit.connect()
	# digit.set_intensity(Digit.LIGHTING_MAX)

	# qvga_res = Digit.STREAMS["QVGA"]
	# digit.set_resolution(qvga_res)

	# fps_30 = Digit.STREAMS["QVGA"]["fps"]["30fps"]
	# digit.set_fps(fps_30)

	# HOST = '127.0.0.1'
	# PORT = 9000 #9000, 9001

	# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	# sock.connect((HOST, PORT))

	# sock.send(str.encode('START' + '\n'))
	# sock.send(str.encode('RESET domain ../experiments/pogo.json' + '\n'))

	# counter = 0
	# while True:
	#     # sock.send(str.encode(str(counter)))
	#     sock.send(str.encode('MOVE W' + '\n'))
	#     counter += 1

	#     data = sock.recv(1024).decode()
	#     print("data recv: ", counter, data)



	# you could name this function
	try:
		main()
	except rospy.ROSInterruptException:
		pass
