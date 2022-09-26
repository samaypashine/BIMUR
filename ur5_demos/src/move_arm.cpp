/**
 * @file move_arm.cpp
 * @author Samay
 * @brief 
 * @version 1.0
 * @date 2022-06-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <ros/ros.h>

#include "joint_recorder/recorderMsg.h"
#include "joint_recorder/recorderSrv.h"

#include <signal.h>
#include <iostream>
#include <random>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include<stdlib.h>

#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <termios.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>


#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include <bimur_robot_vision/TabletopPerception.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/GetPositionIK.h>

#include <robotiq_85_msgs/GripperCmd.h>


/**
 * @brief States of a pose out of the way.
 * name: [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
 * position: [ 1.7994565963745117, -1.304173771535055, 0.46854838728904724, -2.487392250691549, -4.015228335057394, -0.6358125845538538]
 * velocity :  [-0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
 * effort: [1.1388397216796875, 2.6812055110931396, 1.275590181350708, 0.4529372453689575, 0.16012932360172272, 0.06405173242092133]
 */


class ur5Behavior
{
	public:

	ros::NodeHandle n;
	ros::Publisher pose_pub;
	ros::Publisher gripper_pub;
	ros::Publisher command_pub;
	// ros::Publisher digit_pub;
	ros::Publisher realsense_frame_capture_pub;

	ros::ServiceClient ik_client;

	ros::ServiceClient clientObj;
	joint_recorder::recorderSrv srvRequest;

	geometry_msgs::PoseStamped current_button_pose;
	moveit::planning_interface::MoveGroupInterface *group;

	float Z_OFFSET;
	float NEW_Z_OFFSET;
	double Z_coord;
	bool g_caught_sigint;

	pcl::PointCloud<pcl::PointXYZ> target_object;
	std::string frame_id;

	ur5Behavior();
	~ur5Behavior();
	void sig_handler(int sig);
	int getch(void);
	bool close_gripper(double, double, double);
	bool open_gripper(double, double, double);
	void init_pos();
	// pcl::PointCloud<pcl::PointXYZ> detectObjects();
	// geometry_msgs::Pose getPoseFromObject();
	geometry_msgs::Pose getPoseFromCoord(double, double, double);
	double* inverseKinematic(geometry_msgs::Pose, double [], int);
	// int trajectoryChecker(moveit_msgs::RobotTrajectory);
	// void motionPlanner(geometry_msgs::Pose);
	std_msgs::String URScriptCommand(double [], double, double, double, double);
	// void cartesianControl();
	void move_arm();
	// void shakingMotion();
	void stirringMotion_1(std::string);
	void stirringMotion_2(std::string);
	void stirringMotion_3(std::string);
	void stirringMotion_4(std::string);
	void action_position_top();
	void action_position_down();
	void initialize_folders(std::string);
	// void grabObject();
};

ur5Behavior::ur5Behavior()
{
	Z_OFFSET = 0.25;
	NEW_Z_OFFSET = 0.0;
	Z_coord = 0.704;
	g_caught_sigint = false;
    
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_arm_demo/pose", 10);
	gripper_pub = n.advertise<robotiq_85_msgs::GripperCmd>("/gripper/cmd", 10);
	command_pub = n.advertise<std_msgs::String>("/ur_hardware_interface/script_command", 10);
	
	// digit_pub = n.advertise<std_msgs::Bool>("/digit_sensor", 10);
	// realsense_frame_capture_pub = n.advertise<std_msgs::Bool>("/frame_capture", 10);
	ik_client = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");

	clientObj = n.serviceClient<joint_recorder::recorderSrv>("data_recording_service");

	group = new moveit::planning_interface::MoveGroupInterface("manipulator");
	// group->setPlannerId("RRTConfigDefault");
	group->setPlannerId("RRTConnect");
	// group->setPlannerId("RRTConnectkConfigDefault");
    group->setGoalTolerance(0.01);
	group->setPlanningTime(15);

	frame_id = "camera_depth_optical_frame";
	init_pos();
}

ur5Behavior::~ur5Behavior()
{
}

void ur5Behavior::sig_handler(int sig)
{	
	g_caught_sigint = true;
    ROS_ERROR("Caught sigint, SHUTDOWN...");
    ros::shutdown();
    exit(1);
}

int ur5Behavior::getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

bool ur5Behavior::close_gripper(double position = 0.0, double speed = 1.0, double force = 1.0)
{
	robotiq_85_msgs::GripperCmd msg;
	msg.position = position;
	msg.speed = speed;
	msg.force = force;
	gripper_pub.publish(msg);

	ros::spinOnce();	
	return 0;
}

bool ur5Behavior::open_gripper(double position = 1.0, double speed = 1.0, double force = 100.0)
{	
	robotiq_85_msgs::GripperCmd msg;
	msg.position = position;
	msg.speed = speed;
	msg.force = force;
	gripper_pub.publish(msg);

	ros::spinOnce();
	return 0;
}

void ur5Behavior::init_pos()
{
	ROS_INFO("Moving to the initial position.");
	open_gripper();
	std::map<std::string, double> target;
	target["shoulder_pan_joint"] =  0.4166615605354309;
	target["shoulder_lift_joint"] =  -1.6050642172442835;
	target["elbow_joint"] =  2.164191246032715;
	target["wrist_1_joint"] = 3.720980167388916;
	target["wrist_2_joint"] = -4.021045986806051;
	target["wrist_3_joint"] = -0.6397879759417933;
	
	group->setJointValueTarget(target);
	move_arm();
	ros::spinOnce();
}


void ur5Behavior::action_position_top()
{
	ROS_INFO("Moving to the initial position.");
	// open_gripper();
	std::map<std::string, double> target;
	target["shoulder_pan_joint"] =  -0.625642;
	target["shoulder_lift_joint"] =  -0.530822;
	target["elbow_joint"] =  2.27726;
	target["wrist_1_joint"] = 3.49744;
	target["wrist_2_joint"] = -4.10288;
	target["wrist_3_joint"] = -2.34607;

	// X : 0.064, Y : -0.078, Z : 0.556
	// [-0.625642, -0.530822, 2.27726, 3.49744, -4.10288, -2.34607]

	group->setJointValueTarget(target);
	move_arm();
	ros::spinOnce();
}

void ur5Behavior::action_position_down()
{
	ROS_INFO("Moving to the initial position.");
	// open_gripper();
	std::map<std::string, double> target;
	target["shoulder_pan_joint"] =  -0.750101;
	target["shoulder_lift_joint"] =  -0.712315;
	target["elbow_joint"] =  2.25894;
	target["wrist_1_joint"] = 3.76561;
	target["wrist_2_joint"] = -4.16968;
	target["wrist_3_joint"] = -2.22077;

	// X : 0.054, Y : -0.058, Z : 0.636
	// [-0.750101, -0.712315, 2.25894, 3.76561, -4.16968, -2.22077]

	group->setJointValueTarget(target);
	move_arm();
	ros::spinOnce();
}

void ur5Behavior::move_arm()
{
	ROS_INFO("Moving the Arm.");
	group->move();
	ros::spinOnce();
}

void ur5Behavior::initialize_folders(std::string folderName)
{
	std::string command1 = "mkdir -p " + folderName + "color/";
	std::string command2 = "mkdir -p " + folderName + "depth/";
	std::string command3 = "mkdir -p " + folderName + "touch/";
    system(command1.c_str());		
    system(command2.c_str());		
    system(command3.c_str());	
    ROS_INFO("Folder structure created.");	
	
	srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "joint_states.csv";
    srvRequest.request.topic.data = "/joint_states";
    clientObj.call(srvRequest); //set filename/path node1
	ROS_INFO("Called for joint_states");
	// getch();

    srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "gripper_joint_states.csv";
    srvRequest.request.topic.data = "/gripper/joint_states";
	clientObj.call(srvRequest); //set filename/path node2
    ROS_INFO("Called for gripper_joint_states");
	// getch();

	srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "wrench.csv";
    srvRequest.request.topic.data = "/wrench";
    clientObj.call(srvRequest); //set filename/path node3
	ROS_INFO("Called for wrench");
	// getch();

    srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "audio.wav";
    srvRequest.request.topic.data = "audio_capture"; // "topic" here is just for the node to recognize that the command is intended for itself
    clientObj.call(srvRequest);
	ROS_INFO("Called for audio");
	// getch(); //set filename/path node4
    // ROS_INFO("File names set. Ready to start recording motion.");

	srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "color/";
    srvRequest.request.topic.data = "color_frame_capture"; // "topic" here is just for the node to recognize that the command is intended for itself
    clientObj.call(srvRequest); //set filename/path node4
	ROS_INFO("Called for color_frame_capture");
	// getch();
	
	srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "depth/";
    srvRequest.request.topic.data = "depth_frame_capture"; // "topic" here is just for the node to recognize that the command is intended for itself
    clientObj.call(srvRequest); //set filename/path node4
    ROS_INFO("Called for depth_frame_capture");
	// getch();

    srvRequest.request.command.data ="start";
	clientObj.call(srvRequest);
}

geometry_msgs::Pose ur5Behavior::getPoseFromCoord(double x, double y, double z)
{
	geometry_msgs::Pose pose_i;
	pose_i.position.x = x;
	pose_i.position.y = y;
	pose_i.position.z = z;
	pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
	// ROS_INFO_STREAM(pose_i.orientation);

	return pose_i;
}

double* ur5Behavior::inverseKinematic(geometry_msgs::Pose pose_i, double joint[6], int planFlag = 0)
{
	geometry_msgs::PoseStamped stampedPose, stampOut;
	tf::TransformListener listener;
	moveit_msgs::GetPositionIK::Request ik_request;
    moveit_msgs::GetPositionIK::Response ik_response;

	stampedPose.header.frame_id = frame_id;
	stampedPose.header.stamp = ros::Time(0);
	stampedPose.pose = pose_i;
 
	listener.waitForTransform(stampedPose.header.frame_id, "world", ros::Time(0), ros::Duration(3.0));
	listener.transformPose("world", stampedPose, stampOut);

	stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14/2, 3.14/2, 0.0);
	stampOut.pose.position.z += Z_OFFSET;

	ROS_INFO("Publishing the Pose");
	pose_pub.publish(stampOut);

	if (planFlag == 1)
	{
		group->setPoseReferenceFrame(stampOut.header.frame_id);
		group->setPoseTarget(stampOut);
		group->setStartState(*group->getCurrentState());
	}
    
	ik_request.ik_request.group_name = "manipulator";
    ik_request.ik_request.pose_stamped = stampOut;
    ik_client.call(ik_request, ik_response);

	joint[0] = ik_response.solution.joint_state.position[0];
	joint[1] = ik_response.solution.joint_state.position[1];
	joint[2] = ik_response.solution.joint_state.position[2];
	joint[3] = ik_response.solution.joint_state.position[3];
	joint[4] = ik_response.solution.joint_state.position[4];
	joint[5] = ik_response.solution.joint_state.position[5];
	
	ros::spinOnce();
	return joint;
}

std_msgs::String ur5Behavior::URScriptCommand(double joint[], double a = 0.20, double v = 0.20, double t = 0, double r = 0)
{
	std_msgs::String command; 
	command.data = "movej([";

	command.data += std::to_string(joint[0]);
	command.data += ",";

	command.data += std::to_string(joint[1]);
	command.data += ",";

	command.data += std::to_string(joint[2]);
	command.data += ",";

	command.data += std::to_string(joint[3]);
	command.data += ",";

	command.data += std::to_string(joint[4]);
	command.data += ",";

	command.data += std::to_string(joint[5]);
	command.data += "],a=";
	
	command.data += std::to_string(a);
	command.data += ",v=";

	command.data += std::to_string(v);
	command.data += ",t=";

	command.data += std::to_string(t);
	command.data += ",r=";
	
	command.data += std::to_string(r);
	command.data += ")\n";

	return command;
}


void ur5Behavior::stirringMotion_1(std::string base_path)
{	
	ROS_INFO("Motion 1");
	ROS_INFO("Attach the end effector.");
	getch();
	close_gripper();
	getch();

	double temp_joints_c[6];
	double temp_joints_1[6];
	double temp_joints_2[6];
	double temp_joints_3[6];
	double temp_joints_4[6];
	int rotations = 5;
	double a = 1.5;
	double v = 1.5;
	double radius = 0.02;


	ROS_INFO("Moving Above the Container");
	action_position_top();
	getch();

	// Lowering the Arm
	ROS_INFO("Lowering the arm.");
	action_position_down();
	getch();


	double x = 0.054;
	double y = -0.058;
	double z = 0.636;
	// X : 0.054, Y : -0.058, Z : 0.636

	geometry_msgs::Pose pose_c = getPoseFromCoord(x, y, z);
	ROS_INFO("Past get pose");
	double* joint_angles_c = inverseKinematic(pose_c, temp_joints_c);
	ROS_INFO("Past inverse");

	geometry_msgs::Pose pose_1 = getPoseFromCoord(x, y-radius, z);	
	double* joint_angles_1 = inverseKinematic(pose_1, temp_joints_1);

	geometry_msgs::Pose pose_2 = getPoseFromCoord(x+radius, y, z);	
	double* joint_angles_2 = inverseKinematic(pose_2, temp_joints_2);
	
	geometry_msgs::Pose pose_3 = getPoseFromCoord(x, y+radius, z);	
	double* joint_angles_3 = inverseKinematic(pose_3, temp_joints_3);

	geometry_msgs::Pose pose_4 = getPoseFromCoord(x-radius, y, z);	
	double* joint_angles_4 = inverseKinematic(pose_4, temp_joints_4);


	std_msgs::String point_c = URScriptCommand(joint_angles_c, a, v);
	std_msgs::String point_1 = URScriptCommand(joint_angles_1, a, v);
	std_msgs::String point_2 = URScriptCommand(joint_angles_2, a, v);
	std_msgs::String point_3 = URScriptCommand(joint_angles_3, a, v);
	std_msgs::String point_4 = URScriptCommand(joint_angles_4, a, v);

	initialize_folders(base_path + "/motion_1/");
	ros::Duration(2.0).sleep();

	while (rotations > 0)
	{	
		std::cout<<"[ INFO] Rotations : "<<rotations<<"\n";
		
		ROS_INFO_STREAM(point_1);
		command_pub.publish(point_1);
		ros::Duration(0.75).sleep();

		ROS_INFO_STREAM(point_2);
		command_pub.publish(point_2);
		ros::Duration(0.75).sleep();

		ROS_INFO_STREAM(point_3);
		command_pub.publish(point_3);
		ros::Duration(0.75).sleep();

		ROS_INFO_STREAM(point_4);
		command_pub.publish(point_4);
		ros::Duration(0.75).sleep();

		rotations--;
	}

	ROS_INFO("Stopping the data recording node.");
	srvRequest.request.command.data ="stop";
	clientObj.call(srvRequest);

	command_pub.publish(point_c);
	// ros::Duration(10.0).sleep();
}


void ur5Behavior::stirringMotion_2(std::string base_path)
{
	ROS_INFO("Motion 2");

	double temp_joints_c[6];
	int rotations = 5;
	double a = 1.50;
	double v = 1.50;
	double radius = 1.0;

	double x = 0.054;
	double y = -0.058;
	double z = 0.636;

	geometry_msgs::Pose pose_c = getPoseFromCoord(x, y, z);	
	double* joint_angles_c = inverseKinematic(pose_c, temp_joints_c);



	initialize_folders(base_path + "/motion_2/");
	ros::Duration(2.0).sleep();

	while (rotations > 0)
	{	
		std::cout<<"[ INFO] Rotations : "<<rotations<<"\n";

		joint_angles_c[5] += radius;
		std_msgs::String point_2 = URScriptCommand(joint_angles_c, a, v);
		command_pub.publish(point_2);
		ros::Duration(0.75).sleep();
		
		joint_angles_c[5] -= radius;
		point_2 = URScriptCommand(joint_angles_c, a, v);
		command_pub.publish(point_2);
		ros::Duration(0.75).sleep();

		rotations--;
	}

	ROS_INFO("Stopping the data recording node.");
	srvRequest.request.command.data ="stop";
	clientObj.call(srvRequest);
	// ros::Duration(10.0).sleep();
}

void ur5Behavior::stirringMotion_3(std::string base_path)
{
	ROS_INFO("Motion 3");

	double temp_joints_c[6];
	double temp_joints_1[6];
	double temp_joints_2[6];
	double temp_joints_3[6];
	double temp_joints_4[6];
	int rotations = 5;
	double a = 1.50;
	double v = 1.50;
	double radius = 0.02;
	double radius1 = 1.00;
	
	geometry_msgs::Pose current_pose = group->getCurrentPose().pose;
	double x = 0.054;
	double y = -0.058;
	double z = 0.636;
	// X : 0.054, Y : -0.058, Z : 0.636

	geometry_msgs::Pose pose_c = getPoseFromCoord(x, y, z);	
	double* joint_angles_c = inverseKinematic(pose_c, temp_joints_c);

	geometry_msgs::Pose pose_1 = getPoseFromCoord(x, y-radius, z);	
	double* joint_angles_1 = inverseKinematic(pose_1, temp_joints_1);

	geometry_msgs::Pose pose_2 = getPoseFromCoord(x+radius, y, z);	
	double* joint_angles_2 = inverseKinematic(pose_2, temp_joints_2);
	
	geometry_msgs::Pose pose_3 = getPoseFromCoord(x, y+radius, z);	
	double* joint_angles_3 = inverseKinematic(pose_3, temp_joints_3);

	geometry_msgs::Pose pose_4 = getPoseFromCoord(x-radius, y, z);	
	double* joint_angles_4 = inverseKinematic(pose_4, temp_joints_4);
	


	initialize_folders(base_path + "/motion_3/");
	ros::Duration(2.0).sleep();


	while (rotations > 0)
	{	
		std::cout<<"[ INFO] Rotations : "<<rotations<<"\n";
		
		joint_angles_1[5] += radius1;
		std_msgs::String point_1 = URScriptCommand(joint_angles_1, a, v);
		command_pub.publish(point_1);
		joint_angles_1[5] -= radius1;
		ros::Duration(1.5).sleep();

		joint_angles_2[5] -= radius1;
		std_msgs::String point_2 = URScriptCommand(joint_angles_2, a, v);
		command_pub.publish(point_2);
		joint_angles_2[5] += radius1;
		ros::Duration(1.5).sleep();

		joint_angles_3[5] += radius1;
		std_msgs::String point_3 = URScriptCommand(joint_angles_3, a, v);
		command_pub.publish(point_3);
		joint_angles_3[5] -= radius1;
		ros::Duration(1.5).sleep();

		joint_angles_4[5] -= radius1;
		std_msgs::String point_4 = URScriptCommand(joint_angles_4, a, v);
		command_pub.publish(point_4);
		joint_angles_4[5] += radius1;
		ros::Duration(1.5).sleep();

		rotations--;
	}

	ROS_INFO("Stopping the data recording node.");
	srvRequest.request.command.data ="stop";
	clientObj.call(srvRequest);

	std_msgs::String point_c = URScriptCommand(joint_angles_c, a, v);
	command_pub.publish(point_c);
	// ros::Duration(10.0).sleep();
}

void ur5Behavior::stirringMotion_4(std::string base_path)
{
	ROS_INFO("Motion 4");

	double temp_joints_c[6];
	double temp_joints_1[6];
	double temp_joints_2[6];
	double temp_joints_3[6];
	double temp_joints_4[6];
	int rotations = 5;
	double a = 1.50;
	double v = 1.50;
	double radius = 0.02;

	
	geometry_msgs::Pose current_pose = group->getCurrentPose().pose;
	double x = 0.054;
	double y = -0.058;
	double z = 0.636;
	// X : 0.054, Y : -0.058, Z : 0.636

	geometry_msgs::Pose pose_c = getPoseFromCoord(x, y, z);	
	double* joint_angles_c = inverseKinematic(pose_c, temp_joints_c);

	geometry_msgs::Pose pose_1 = getPoseFromCoord(x, y-radius, z);	
	double* joint_angles_1 = inverseKinematic(pose_1, temp_joints_1);

	geometry_msgs::Pose pose_2 = getPoseFromCoord(x+radius, y, z);	
	double* joint_angles_2 = inverseKinematic(pose_2, temp_joints_2);
	
	geometry_msgs::Pose pose_3 = getPoseFromCoord(x, y+radius, z);	
	double* joint_angles_3 = inverseKinematic(pose_3, temp_joints_3);

	geometry_msgs::Pose pose_4 = getPoseFromCoord(x-radius, y, z);	
	double* joint_angles_4 = inverseKinematic(pose_4, temp_joints_4);


	std_msgs::String point_c = URScriptCommand(joint_angles_c, a, v);
	std_msgs::String point_1 = URScriptCommand(joint_angles_1, a, v);
	std_msgs::String point_2 = URScriptCommand(joint_angles_2, a, v);
	std_msgs::String point_3 = URScriptCommand(joint_angles_3, a, v);
	std_msgs::String point_4 = URScriptCommand(joint_angles_4, a, v);
	


	initialize_folders(base_path + "/motion_4/");
	ros::Duration(2.0).sleep();

	while (rotations > 0)
	{	
		std::cout<<"[ INFO] Rotations : "<<rotations<<"\n";
		
		// ROS_INFO_STREAM(point_1);
		command_pub.publish(point_c);
		ros::Duration(0.90).sleep();

		command_pub.publish(point_1);
		ros::Duration(0.90).sleep();

		command_pub.publish(point_3);
		ros::Duration(0.90).sleep();

		command_pub.publish(point_c);
		ros::Duration(0.90).sleep();

		command_pub.publish(point_4);
		ros::Duration(0.90).sleep();

		command_pub.publish(point_2);
		ros::Duration(0.90).sleep();

		rotations--;
	}

	ROS_INFO("Stopping the data recording node.");
	srvRequest.request.command.data ="stop";
	clientObj.call(srvRequest);

	command_pub.publish(point_c);
	// ros::Duration(10.0).sleep();
	open_gripper();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_arm");

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ur5Behavior Obj;

	Obj.stirringMotion_1("/home/pc1/Downloads/Liquid_Dataset/temp");
	// ROS_INFO("Time out between motions");
	// ros::Duration(20.0).sleep();
	Obj.stirringMotion_2("/home/pc1/Downloads/Liquid_Dataset/temp");
	// ROS_INFO("Time out between motions");
	// ros::Duration(20.0).sleep();
	Obj.stirringMotion_3("/home/pc1/Downloads/Liquid_Dataset/temp");
	// ROS_INFO("Time out between motions");
	// ros::Duration(30.0).sleep();
	Obj.stirringMotion_4("/home/pc1/Downloads/Liquid_Dataset/temp");
	// ROS_INFO("Time out between motions");
	// ros::Duration(20.0).sleep();
	spinner.stop();

	return 0;
}




























