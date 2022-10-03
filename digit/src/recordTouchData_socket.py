import time
import argparse
from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler
import socket


# Server
HOST = '127.0.0.1'
PORT = 9000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(0)
conn, addr = s.accept()
print('conn', conn)
print('addr', addr)
# if conn:
print('Connected by', addr)
while True:
	# time.sleep(1)
	data = conn.recv(1024).decode()
	data = str(data)
	print("data: ", data)

	if data == 'record':
		print("SAVING file")

	# conn.sendall(str.encode(data))



# parser = argparse.ArgumentParser()
# parser.add_argument("--num", help="Filename", type=int)
# args = parser.parse_args()

# digit = Digit("D20501", "Left Gripper")
# print("digit: ", digit.dev_name)
# digit.connect()
# digit.set_intensity(Digit.LIGHTING_MAX)

# qvga_res = Digit.STREAMS["QVGA"]
# digit.set_resolution(qvga_res)

# fps_30 = Digit.STREAMS["QVGA"]["fps"]["30fps"]
# digit.set_fps(fps_30)

# frame = digit.save_frame("/home/pc1/Downloads/Liquid_Dataset/touch_data/{}.jpg".format(args.num))
# print("Frame Saved at the location by name : {}.jpg".format(args.num))

