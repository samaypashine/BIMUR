# bimur_robot_vision

Launch the camera:

`roslaunch astra_launch astra.launch`

Run the bimur_robot_vision package:

`rosrun bimur_robot_vision object_detection_node`

Call the service:

`rosservice call /bimur_object_detector/detect “{}”`
