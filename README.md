# Bimur

Bimur (Bi-manual UR5). This repository is for packages related to UR5 at MuLIP Lab, Tufts.

## Requirements

1. Ubuntu 18.04
2. ROS Melodic
3. Run `./install.sh`

### Start microphone
- Make sure the ReSpeaker Microphone Array is plugged into the computer vis USB <br>
- Check audio device name: `arecord -l` <br>
- Install microphone driver: https://github.com/furushchev/respeaker_ros <br>
`roslaunch respeaker_ros respeaker.launch`

## Manipulation

## To perform tool behaviors and record sensory data:

- Check camera feed before recording (`rosrun image_view image_view image:=/camera/color/image_raw`).
- Hold the container using left arm gripper without turning the left arm on. If needed turn on the left arm to set up the arm and gripper position, and then turn it off.
- Make sure the lights in the lab are at full intensity.
- Place the containers tightly completely inside the left gripper.
- Before placing shake the container to level the contents inside.
- Use liquid objects in the end after finishing all the objects.
- Finish one trial with all the objects using one tool, then switch to the next tool and repeat until all the tools are used. Then, repeat the same for the next trial.
- Check touch sensor recorded images. If images are glitchy, restart the sensor and record again.
- After each trial, clean the tool and table if needed.

```
sudo chmod 777 /dev/ttyUSB0

roslaunch bimur_bringup right_robot_moveit.launch robot_ip:=172.22.22.2

roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

roslaunch respeaker_ros respeaker.launch

roslaunch joint_recorder recordingService.launch numTopics:=3 topicName1:=/joint_states topicName2:=/gripper/joint_states topicName3:=/wrench

python src/UR5-ros-melodic/digit/src/recordTouchData_json.py

python data_processing/make_videos.py

rosrun image_view image_view image:=/camera/color/image_raw

rosrun ur5_demos tool_behaviors

```
