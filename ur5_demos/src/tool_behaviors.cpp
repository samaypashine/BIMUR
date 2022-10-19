/**
 * @file tool_behaviors.cpp
 * @author Samay, Gyan
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

#include <stdlib.h>

#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

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

int countFiles(std::string base_path) {

	ROS_INFO("Counting fils in path: %s.\n", base_path.c_str());

	DIR *dp;
	int i = 0;
	struct dirent *ep;
	dp = opendir(base_path.c_str());

	if (dp != NULL) {
		while (ep = readdir (dp)) {
			i++;
		}
		(void) closedir (dp);
	}
	else {
		perror("Couldn't open the directory");
		return 0;
	}

	i = i - 2;

	return i;
}

bool tryParse(std::string& input, int& output) {
    try{
        output = std::stoi(input);
    } catch (std::invalid_argument) {
        return false;
    }
    return true;
}

std::string getLabelFromUser(std::vector<std::string> labels) {

	std::sort(labels.begin(), labels.end());

    printf("\nLabels: IDs\n");
    for (int i=0; i < labels.size(); i++) {
   		std::cout << labels[i] << ": " << i << "\n";
	}

	std::string input;
    int x;

    getline(std::cin, input);

    while (!tryParse(input, x)) {
        std::cout << "Bad entry. Enter a NUMBER: ";
        getline(std::cin, input);
    }

    std::cout << "You Selected: " << labels[x] << "\n";

    return labels[x];
}

long getFileSize(std::string filename) {

    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

void checkData(std::string base_path) {

	bool dataIssue = false;

	std::string path = base_path;
	int c = countFiles(path);
	if (c <= 3) {
		ROS_ERROR("DATA DID NOT RECORD! There's %d files in %s.", c, path.c_str());
		dataIssue = true;
	}

	path = base_path + "camera_rgb_image/";
	c = countFiles(path);
	if (c <= 10) {
		ROS_ERROR("CAMERA DATA DID NOT RECORD! There's %d files in %s.", c, path.c_str());
		dataIssue = true;
	}

	path = base_path + "touch_image/";
	c = countFiles(path);
	if (c <= 10) {
		ROS_ERROR("TOUCH DATA DID NOT RECORD! There's %d files in %s.", c, path.c_str());
		dataIssue = true;
	}

	path = base_path + "audio.wav";
	long s = getFileSize(path) / 1024;  // KB
	if (s < 1) {
		ROS_ERROR("AUDIO DATA DID NOT RECORD! Audio file size is %d in %s.", s, path.c_str());
		dataIssue = true;
	}

	if (dataIssue) {

		base_path = base_path.substr(0, base_path.length()-2); // removing the last /

		const size_t last_slash_idx = base_path.rfind("/"); // finding idx of /after trial-X folder
		if (std::string::npos != last_slash_idx) {
			base_path = base_path.substr(0, last_slash_idx); // removing behavior folder
		}

		ROS_ERROR("Do you want to delete the data in %s?", base_path.c_str());
		std::vector<std::string> options = {"yes", "no"};
		std::string ans = getLabelFromUser(options);

		if (ans.compare("yes") == 0) {
			std::string command = "rm -r " + base_path;
    		system(command.c_str());
		}
	}
}

class ur5Behavior {
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
	double* inverseKinematic(geometry_msgs::Pose, int);
	// int trajectoryChecker(moveit_msgs::RobotTrajectory);
	// void motionPlanner(geometry_msgs::Pose);
	std_msgs::String URScriptCommand(double [], double, double, double, double);
	// void cartesianControl();
	void move_arm();
	// void shakingMotion();
	void lookBehavior(std::string, double, double, double);
	void stirringBehavior_1(std::string, double , double, double, int, int, double);
	void stirringBehavior_2(std::string, double , double, double, int, double);
	void stirringBehavior_3(std::string, double , double, double, double, int, int, double);
	void stirringBehavior_4(std::string, double , double, double, int, int, double);
	double** getJointAnglesOnCircularPoints(double, double, double, double, int);
	void action_position_top();
	void action_position_down();
	void startRecording(std::string);
	// void checkData(std::string);
	// void grabObject();
};

ur5Behavior::ur5Behavior() {
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

ur5Behavior::~ur5Behavior() {
}

void ur5Behavior::sig_handler(int sig) {	
	g_caught_sigint = true;
    ROS_ERROR("Caught sigint, SHUTDOWN...");
    ros::shutdown();
    exit(1);
}

int ur5Behavior::getch(void) {
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

bool ur5Behavior::close_gripper(double position = 0.0, double speed = 100.0, double force = 100.0) {
	robotiq_85_msgs::GripperCmd msg;
	msg.position = position;
	msg.speed = speed;
	msg.force = force;
	gripper_pub.publish(msg);

	ros::spinOnce();	
	return 0;
}

bool ur5Behavior::open_gripper(double position = 1.0, double speed = 1.0, double force = 100.0) {
	robotiq_85_msgs::GripperCmd msg;
	msg.position = position;
	msg.speed = speed;
	msg.force = force;
	gripper_pub.publish(msg);

	ros::spinOnce();
	return 0;
}

void ur5Behavior::init_pos() {
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

void ur5Behavior::action_position_top() {
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

void ur5Behavior::action_position_down() {
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

void ur5Behavior::move_arm() {
	ROS_INFO("Moving the Arm.");
	group->move();
	ros::spinOnce();
}

void ur5Behavior::startRecording(std::string folderName) {
	std::string command1 = "mkdir -p " + folderName + "camera_rgb_image/";
	std::string command2 = "mkdir -p " + folderName + "camera_depth_image/";
	std::string command3 = "mkdir -p " + folderName + "touch_image/";
    system(command1.c_str());		
    system(command2.c_str());		
    system(command3.c_str());	
    ROS_INFO("Folder structure created.");	
	
	srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "joint_states.csv";
    srvRequest.request.topic.data = "/joint_states";
    clientObj.call(srvRequest); //set filename/path node1
	ROS_INFO("Called for joint_states");

    srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "gripper_joint_states.csv";
    srvRequest.request.topic.data = "/gripper/joint_states";
	clientObj.call(srvRequest); //set filename/path node2
    ROS_INFO("Called for gripper_joint_states");

	srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "wrench.csv";
    srvRequest.request.topic.data = "/wrench";
    clientObj.call(srvRequest); //set filename/path node3
	ROS_INFO("Called for wrench");

    srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "audio.wav";
    srvRequest.request.topic.data = "audio_capture"; // "topic" here is just for the node to recognize that the command is intended for itself
    clientObj.call(srvRequest);
	ROS_INFO("Called for audio");

	srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "camera_rgb_image/";
    srvRequest.request.topic.data = "color_frame_capture"; // "topic" here is just for the node to recognize that the command is intended for itself
    clientObj.call(srvRequest); //set filename/path node4
	ROS_INFO("Called for color_frame_capture");
	
	srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "camera_depth_image/";
    srvRequest.request.topic.data = "depth_frame_capture"; // "topic" here is just for the node to recognize that the command is intended for itself
    clientObj.call(srvRequest); //set filename/path node4
    ROS_INFO("Called for depth_frame_capture");
	// getch();

	srvRequest.request.command.data = "set_file_name";
    srvRequest.request.fileName.data = folderName + "touch_image/";
    srvRequest.request.topic.data = "touch_frame_capture"; // "topic" here is just for the node to recognize that the command is intended for itself
    clientObj.call(srvRequest); //set filename/path node5
    ROS_INFO("Called for touch_frame_capture");

    srvRequest.request.command.data ="start";
	clientObj.call(srvRequest);
}

geometry_msgs::Pose ur5Behavior::getPoseFromCoord(double x, double y, double z) {
	geometry_msgs::Pose pose_i;
	pose_i.position.x = x;
	pose_i.position.y = y;
	pose_i.position.z = z;
	pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
	// ROS_INFO_STREAM(pose_i.orientation);

	return pose_i;
}

double* ur5Behavior::inverseKinematic(geometry_msgs::Pose pose_i, int planFlag = 0) {
	static double joint[6];

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

	if (planFlag == 1) {
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

std_msgs::String ur5Behavior::URScriptCommand(double joint[], double a = 0.20, double v = 0.20, double t = 0, double r = 0) {
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

double** ur5Behavior::getJointAnglesOnCircularPoints(double x_center, double y_center, double z_center, double radius, int numPoints) {
	double angle_per_point = 360 / numPoints;
	double* joint_angles_ptr[numPoints + 1];
	double** joint_angles;

	geometry_msgs::Pose poses;

	joint_angles = new double*[numPoints + 1];	
	joint_angles[0] = new double[6];

	poses = getPoseFromCoord(x_center, y_center, z_center);
	joint_angles_ptr[0] = inverseKinematic(poses);

	for (int k = 0; k < 6; k++)
		joint_angles[0][k] = joint_angles_ptr[0][k];

	for (int i = 1; i <= numPoints; i++) {
		double Y = (y_center + radius * sin((i*angle_per_point * 3.14159) / 180));
		double X = (x_center + radius * cos((i*angle_per_point * 3.14159) / 180));
		
		joint_angles[i] = new double[6];
		poses = getPoseFromCoord(X, Y, z_center);		
		joint_angles_ptr[i] = inverseKinematic(poses);

		for (int k = 0; k < 6; k++) 
			joint_angles[i][k] = joint_angles_ptr[i][k];
	}

	return joint_angles;
}

void ur5Behavior::lookBehavior(std::string base_path, double interval_time = 1.0, double a = 1.5, double v = 1.5) {

	ROS_INFO("Inside Look Behavior.");
	double x = 0.054;
	double y = -0.058;
	double z = 0.66;  // 0.636
	
	geometry_msgs::Pose pose = getPoseFromCoord(x, y, z);
	double* joint_angles_ptr;
	double joint_angles[6];

	joint_angles_ptr = inverseKinematic(pose);

	for (int k = 0; k < 6; k++)
		joint_angles[k] = joint_angles_ptr[k];

	std_msgs::String point = URScriptCommand(joint_angles, a, v);
	ROS_INFO("Moving to Target Position.");
	command_pub.publish(point);
	ros::Duration(1).sleep();

    startRecording(base_path);
	ros::Duration(interval_time).sleep();

    ROS_INFO("Stopping the data recording node.");
	srvRequest.request.command.data = "stop";
	clientObj.call(srvRequest);
}

void ur5Behavior::stirringBehavior_1(std::string base_path, double a , double v, double radius, int rotations = 5, int numPoints = 10, double timeDelay = 0.275) {

	ROS_INFO("Strring Behavior 1");
	ROS_INFO("Attach the end effector.");
	getch();
	close_gripper();
	getch();

	ROS_INFO("Moving Above the Container");
	action_position_top();
	getch();

	ROS_INFO("Lowering the arm.");
	action_position_down();
	getch();

	double x = 0.054;
	double y = -0.058;
	double z = 0.66;  // 0.636
	// X : 0.054, Y : -0.058, Z : 0.636

	double** joint_angles = getJointAnglesOnCircularPoints(x, y, z, radius, numPoints);

	std_msgs::String points[numPoints + 1];
	for (int i = 0; i <= numPoints; i++) {
		points[i] = URScriptCommand(joint_angles[i], a, v);
	}

	startRecording(base_path);
	ros::Duration(0.5).sleep();

	while (rotations > 0) {	
		std::cout<<"[ INFO] Rotations : "<<rotations<<"\n";
		for (int i = 1; i <= numPoints; i++) {
			command_pub.publish(points[i]);
			ros::Duration(timeDelay).sleep();	
		}
		rotations--;
	}

	ROS_INFO("Stopping the data recording node.");
	srvRequest.request.command.data ="stop";
	clientObj.call(srvRequest);

	command_pub.publish(points[0]);
}

void ur5Behavior::stirringBehavior_2(std::string base_path, double a , double v, double radius, int rotations, double timeDelay) {

	close_gripper();
	ROS_INFO("Strring Behavior 2");

	double x = 0.054;
	double y = -0.058;
	double z = 0.66;  // 0.636

	geometry_msgs::Pose pose_c = getPoseFromCoord(x, y, z);	
	double* joint_angles_c = inverseKinematic(pose_c);

	std_msgs::String point[2];
	joint_angles_c[5] += radius;
	point[0] = URScriptCommand(joint_angles_c, a, v);

	joint_angles_c[5] -= radius;
	point[1] = URScriptCommand(joint_angles_c, a, v);

	startRecording(base_path);
	ros::Duration(0.5).sleep();

	while (rotations > 0) {	
		std::cout<<"[ INFO] Rotations : "<<rotations<<"\n";
		for (int i = 0 ; i < 2 ; i++) {
			command_pub.publish(point[i]);
			ros::Duration(timeDelay).sleep();
		}
		rotations--;
	}

	ROS_INFO("Stopping the data recording node.");
	srvRequest.request.command.data ="stop";
	clientObj.call(srvRequest);
}

void ur5Behavior::stirringBehavior_3(std::string base_path, double a , double v, double radius, double twist_angle, int rotations = 5, int numPoints = 10, double timeDelay = 0.275) {

	close_gripper();
	ROS_INFO("Strring Behavior 3");
	
	double x = 0.054;
	double y = -0.058;
	double z = 0.66;  // 0.636

	// X : 0.054, Y : -0.058, Z : 0.636

	double** joint_angles = getJointAnglesOnCircularPoints(x, y, z, radius, numPoints);
	std_msgs::String points_c = URScriptCommand(inverseKinematic(getPoseFromCoord(x, y, z)), a, v);

	startRecording(base_path);
	ros::Duration(0.5).sleep();

	while (rotations > 0) {	
		std::cout<<"[ INFO] Rotations : "<<rotations<<"\n";
	
		for (int i = 1; i <= numPoints ; i++) {
			if (i % 2 != 0)
				joint_angles[i][5] += twist_angle;
			else
				joint_angles[i][5] -= twist_angle;
			
			std_msgs::String points = URScriptCommand(joint_angles[i], a, v);
			command_pub.publish(points);
			ros::Duration(timeDelay).sleep();

			if (i % 2 != 0)
				joint_angles[i][5] -= twist_angle;
			else
				joint_angles[i][5] += twist_angle;
		}	
		rotations--;
	}

	ROS_INFO("Stopping the data recording node.");
	srvRequest.request.command.data ="stop";
	clientObj.call(srvRequest);

	command_pub.publish(points_c);
}

void ur5Behavior::stirringBehavior_4(std::string base_path, double a , double v, double radius, int rotations, int numPoints, double timeDelay) {

	close_gripper();
	ROS_INFO("Strring Behavior 4");
	
	double x = 0.054;
	double y = -0.058;
	double z = 0.66;  // 0.636

	double** joint_angles = getJointAnglesOnCircularPoints(x, y, z, radius, numPoints);
	std_msgs::String points[numPoints + 1];
	for (int i = 0; i <= numPoints; i++) {
		points[i] = URScriptCommand(joint_angles[i], a, v);
	}
	
	startRecording(base_path);
	ros::Duration(0.5).sleep();

	while (rotations > 0) {	
		std::cout << "[ INFO] Rotations : " << rotations << "\n";
		
		for (int i = 1; i <= numPoints; i++) {
			// command_pub.publish(points[0]);
			// ros::Duration(0.90).sleep();

			command_pub.publish(points[i]);
			ros::Duration(0.90).sleep();

			int j = (i + numPoints/2) % numPoints;
			if (j == 0) {
				command_pub.publish(points[numPoints]);
			}
			else {
				command_pub.publish(points[j]);
			}
			ros::Duration(0.90).sleep();			
		}
		rotations--;
	}

	ROS_INFO("Stopping the data recording node.");
	srvRequest.request.command.data ="stop";
	clientObj.call(srvRequest);

	command_pub.publish(points[0]);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "move_arm");
	ros::AsyncSpinner spinner(0);
	spinner.start();

	std::string sensorDataPath = "/home/pc1/Downloads/Liquid_Dataset/";

	auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
    auto curr_time = oss.str();
    std::cout << "Time Stamp: " << curr_time << std::endl;

    std::string robotName = "ur5";
    std::vector<std::string> tools = {"placticspoon", "metalspoon"};
    std::vector<std::string> contents = {"chickpea", "wheat"};

    std::string tool = getLabelFromUser(tools);
    std::string content = getLabelFromUser(contents);
    std::string objectName = robotName + "_" + tool + "_" + content;
    std::cout << "objectName: " << objectName << "\n";

    int trialNo = countFiles(sensorDataPath + objectName + "/");
    std::cout << "trialNo: " << trialNo << "\n";

    std::string trialNoStr = std::to_string(trialNo);

    sensorDataPath += objectName + "/" + "trial-" + trialNoStr + "_" + curr_time + "/";
    std::cout << "sensorDataPath: " << sensorDataPath << "\n";

	ur5Behavior Obj;

	std::string behaviorName = "behavior-1-look";
	Obj.lookBehavior(sensorDataPath + behaviorName + "/", 1.0);
	ros::Duration(2.0).sleep();
	checkData(sensorDataPath + behaviorName + "/");
	
	behaviorName = "behavior-2-slow";
	Obj.stirringBehavior_1(sensorDataPath + behaviorName + "/", 0.1, 0.1, 0.025, 5, 10, 1);
	ros::Duration(2.0).sleep();
	checkData(sensorDataPath + behaviorName + "/");

	behaviorName = "behavior-2-fast";
	Obj.stirringBehavior_1(sensorDataPath + behaviorName + "/", 1, 1, 0.025, 5, 10, 0.4);
	ros::Duration(2.0).sleep();
	checkData(sensorDataPath + behaviorName + "/");

	// behaviorName = "behavior-3";
	// Obj.stirringBehavior_2(sensorDataPath + behaviorName + "/", 1.5, 1.5, 1.0, 5, 0.75);
	// ros::Duration(2.0).sleep();
	// checkData(sensorDataPath + behaviorName + "/");

	// behaviorName = "behavior-4";
	// Obj.stirringBehavior_3(sensorDataPath + behaviorName + "/", 1.5, 1.5, 0.02, 1.0, 5, 10, 1.5);
	// ros::Duration(2.0).sleep();
	// checkData(sensorDataPath + behaviorName + "/");

	// behaviorName = "behavior-5";
	// Obj.stirringBehavior_4(sensorDataPath + behaviorName + "/", 1.5, 1.5, 0.02, 5, 10, 2);
	// ros::Duration(2.0).sleep();
	// checkData(sensorDataPath + behaviorName + "/");

	Obj.open_gripper();
	spinner.stop();

	return 0;

	/*
	TODO:
	X Count no. of files in a folder to check recorded data
	X Add time stamp to folder
	X Ask user for object id (e.g. tool, contect)
	X Compute trial no.
	X Check audio file size

	Ask user if trial is okay
	*/

}
