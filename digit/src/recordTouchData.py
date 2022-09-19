import time
import argparse
from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler


parser = argparse.ArgumentParser()
parser.add_argument("--num", help="Filename", type=int)
args = parser.parse_args()

digit = Digit("D20501", "Left Gripper")
digit.connect()
digit.set_intensity(Digit.LIGHTING_MAX)

qvga_res = Digit.STREAMS["QVGA"]
digit.set_resolution(qvga_res)

fps_30 = Digit.STREAMS["QVGA"]["fps"]["30fps"]
digit.set_fps(fps_30)

frame = digit.save_frame("/home/samay/Desktop/dataset_liquid/touch_data/{}.jpg".format(args.num))
print("Frame Saved at the location by name : {}.jpg".format(args.num))

