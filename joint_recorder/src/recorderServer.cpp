#include "ros/ros.h"
#include "joint_recorder/recorderSrv.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream> 
#include <cstdlib>
#include "joint_recorder/recorderMsg.h"

ros::Publisher controlPub;

bool recordingService(joint_recorder::recorderSrv::Request &req, joint_recorder::recorderSrv::Response &res) {    
    ros::NodeHandle n;
    ros::Publisher controlPub = n.advertise<joint_recorder::recorderMsg>("recording_control_topic",10);
    joint_recorder::recorderMsg msg;
    msg.command.data = req.command.data; 
    msg.fileName.data = req.fileName.data;
    msg.topic.data = req.topic.data;
    
    controlPub.publish(msg);
    // ROS_ERROR("Recorder Server: Command sent to recorder nodes. Service Executed.");

    return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "haptic_record_server");
  ros::NodeHandle n;
  ros::Publisher controlPub = n.advertise<joint_recorder::recorderMsg>("recording_control_topic",10);

  ros::ServiceServer dataRecordingService = n.advertiseService("data_recording_service", recordingService);
  // ROS_ERROR("Data Recording Server: Ready to receive commands");

  while (ros::ok()) {
    ros::spin();
  }

  return 0;
}
