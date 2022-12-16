
import time
import json

from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler


digit = Digit("D20521", "Left Gripper")  # D20501
print("digit: ", digit.dev_name)
digit.connect()
digit.set_intensity(Digit.LIGHTING_MAX)

# qvga_res = Digit.STREAMS["VGA"]
qvga_res = Digit.STREAMS["QVGA"]
digit.set_resolution(qvga_res)

# fps = Digit.STREAMS["VGA"]["fps"]["15fps"]
fps = Digit.STREAMS["QVGA"]["fps"]["30fps"]
digit.set_fps(fps)

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
				# Skipping a few frames
				for _ in range(5):
					img_name = record_info["PATH"] + f'{"0":05}' + ".jpg"
					frame = digit.save_frame(img_name)
			img_name = record_info["PATH"] + f'{curr_count:05}' + ".jpg"
			frame = digit.save_frame(img_name)
			print("Frame Saved at the location: ", img_name)

			curr_count += 1
	except Exception as e:
		print(e)
		digit = Digit("D20501", "Left Gripper")
		print("digit: ", digit.dev_name)
		digit.connect()
		digit.set_intensity(Digit.LIGHTING_MAX)
		digit.set_resolution(qvga_res)
		digit.set_fps(fps)
