#include "ros/ros.h"
#include "joint_recorder/recorderMsg.h"
#include "joint_recorder/recorderSrv.h"


int main (int argc, char **argv) {
    ros::init(argc, argv,"recorder_client_tester");
    ros::NodeHandle n; 
    ros::ServiceClient clientObj = n.serviceClient<joint_recorder::recorderSrv>("data_recording_service");
    joint_recorder::recorderSrv srvRequest;
    // srvRequest.request.command.data = "set_file_name";
    // srvRequest.request.fileName.data = "/home/samay/Desktop/joint_states.csv";
    // srvRequest.request.topic.data = "/joint_states";
    // clientObj.call(srvRequest); //set filename/path node1

    // srvRequest.request.command.data = "set_file_name";
    // srvRequest.request.fileName.data = "/home/samay/Desktop/gripper_joint_states.csv";
    // srvRequest.request.topic.data = "/gripper/joint_states";
	// clientObj.call(srvRequest); //set filename/path node2
    
	// srvRequest.request.command.data = "set_file_name";
    // srvRequest.request.fileName.data = "/home/samay/Desktop/wrench.csv";
    // srvRequest.request.topic.data = "/wrench";
    // clientObj.call(srvRequest); //set filename/path node3

    srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = "/home/samay/Desktop/audio.wav";
    srvRequest.request.topic.data = "audio_capture"; // "topic" here is just for the node to recognize that the command is intended for itself
    clientObj.call(srvRequest); //set filename/path node4
    ROS_INFO("File names set. Ready to start recording motion.");

    srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = "/home/samay/Desktop/";
    srvRequest.request.topic.data = "frame_capture"; // "topic" here is just for the node to recognize that the command is intended for itself
    clientObj.call(srvRequest); //set filename/path node4
    ROS_INFO("File names set. Ready to start recording motion.");

    srvRequest.request.command.data ="start";

    // ROS_ERROR("FILENAMES SET, Waiting 10 s to start recording:");
    // ros::Duration dur(20);
    clientObj.call(srvRequest); //set filename/path node4
    // ROS_INFO("Service calls executed, All nodes should be recording for 10 seconds");
    // dur.sleep();
    // srvRequest.request.command.data ="stop";
    // clientObj.call(srvRequest); //set filename/path node4

    // srvRequest.request.command.data ="shutdown";
    // clientObj.call(srvRequest); //set filename/path node4
    // ROS_ERROR("NODES SHUTDOWN");
    return 0;
}
