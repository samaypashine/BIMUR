#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import time

import demo
from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler

def digit_sensor_callback(flag):
    digit = Digit("D20501", "Left Gripper")
    digit.connect()
    digit.set_intensity(Digit.LIGHTING_MAX)

    qvga_res = Digit.STREAMS["QVGA"]
    digit.set_resolution(qvga_res)

    fps_30 = Digit.STREAMS["QVGA"]["fps"]["30fps"]
    digit.set_fps(fps_30)

    print(digit.info())

    start = time.time()
    diff = time.time() - start
    while diff <= 10:
        frame = digit.save_frame("/home/samay/Desktop/dataset_liquid/touch_data/{}.jpg".format(diff))
        print("Frame Saved at the location by name : {}.jpg".format(diff))
        diff = time.time() - start



if __name__ == "__main__":
    rospy.init_node('digitSensor', anonymous=True)

    rospy.Subscriber("/digit_sensor", Bool, digit_sensor_callback)
    rospy.spin()
