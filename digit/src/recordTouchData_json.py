
import time
import json

from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler


digit = Digit("D20501", "Left Gripper")
print("digit: ", digit.dev_name)
digit.connect()
digit.set_intensity(Digit.LIGHTING_MAX)

qvga_res = Digit.STREAMS["QVGA"]
digit.set_resolution(qvga_res)

fps_30 = Digit.STREAMS["QVGA"]["fps"]["30fps"]
digit.set_fps(fps_30)

record_info_path = "src/UR5-ros-melodic/digit/src/record_info.json"
curr_path = ""

while True:
	time.sleep(0.01)
	try:
		with open(record_info_path, 'r') as openfile:
			record_info = json.load(openfile)
			# print("record_info: ", record_info)

		if record_info["PATH"] and record_info["RECORD"]:
			if curr_path != record_info["PATH"]:
				curr_path = record_info["PATH"]
				curr_count = 0
			frame = digit.save_frame(record_info["PATH"] + str(curr_count) + ".jpg")
			print("Frame Saved at the location: ", record_info["PATH"] + str(curr_count) + ".jpg")

			curr_count += 1
	except Exception as e:
		print(e)
		digit = Digit("D20501", "Left Gripper")
		print("digit: ", digit.dev_name)
		digit.connect()
		digit.set_intensity(Digit.LIGHTING_MAX)

		qvga_res = Digit.STREAMS["QVGA"]
		digit.set_resolution(qvga_res)

		fps_30 = Digit.STREAMS["QVGA"]["fps"]["30fps"]
		digit.set_fps(fps_30)

