#!/usr/bin/env python

import sys
import os
import socket
import json

import rospy
import rospkg
from joint_recorder.msg import recorderMsg


def reset_record_info(path):
	record_info = {"PATH": "", "RECORD": False}
	with open(path, "w") as outfile:
		json.dump(record_info, outfile)


def callback(msg):
	
	# print the actual message in its raw format
	# rospy.loginfo("Here's what was subscribed:", msg.fileName.data, msg.command.data, msg.topic.data)
	
	# otherwise simply print a convenient message on the terminal
	# print('Data from /recording_control_topic received', msg.fileName.data, msg.command.data, msg.topic.data)

	global folderName, record_info_path
	if msg.command.data == "set_file_name" and msg.topic.data == thisNodeName:
		rospy.loginfo("Setted the File name.")
		folderName = msg.fileName.data
		# sock.send(str.encode("PATH:" + folderName))
		with open(record_info_path, 'r') as openfile:
			record_info = json.load(openfile)
			record_info["PATH"] = folderName
		with open(record_info_path, "w") as outfile:
			json.dump(record_info, outfile)
	elif folderName:
		if msg.command.data == "start":
			rospy.loginfo("In Start condition of recordService callback function")

			now = rospy.Time.now()
			print("now: ", str(now.secs) + '.' + str(now.nsecs))

			# sock.send(str.encode("RECORD:" + str(now.secs) + '.' + str(now.nsecs)))

			with open(record_info_path, 'r') as openfile:
				record_info = json.load(openfile)
				record_info["RECORD"] = True
			with open(record_info_path, "w") as outfile:
				json.dump(record_info, outfile)

			# frame = digit.save_frame("/home/pc1/Downloads/Liquid_Dataset/touch_data/{}.jpg".format(args.num))

		elif msg.command.data == "stop":
			rospy.loginfo("In STOP condition of recordService callback function")
			folderName = ""
			# sock.send(str.encode("STOP:"))
			reset_record_info(record_info_path)
		elif msg.command.data == "shutdown":
			rospy.logerr("Shutting down")
			rospy.on_shutdown()

			reset_record_info(record_info_path)
		else:
			rospy.logerr("Command Should be \"set_file_name\" \"start\" or \"stop\" or or \"shutdown\"")


def main():
	
	# initialize a node by the name 'listener'.
	# you may choose to name it however you like,
	# since you don't have to use it ahead

	
	rospy.init_node('touch_frame_recorder_node', anonymous=True)
	rospy.Subscriber("recording_control_topic", recorderMsg, callback)
	
	# spin() simply keeps python from
	# exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	
	thisNodeName = "touch_frame_capture"
	folderName = ""

	rospack = rospkg.RosPack()

	digit_path = rospack.get_path('digit')
	record_info_path = digit_path + os.sep + "src/record_info.json"
	rospy.loginfo(record_info_path)

	# record_info = {"PATH": "", "RECORD": False}
	# 
	# with open(record_info_path, "w") as outfile:
	# 	json.dump(record_info, outfile)

	reset_record_info(record_info_path)

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
