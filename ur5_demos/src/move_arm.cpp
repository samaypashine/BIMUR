/**
 * @file move_arm.cpp
 * @author Willy, Samay
 * @brief 
 * @version 1.0
 * @date 2022-06-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <random>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>


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
	ros::ServiceClient ik_client;
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
	bool close_gripper();
	bool open_gripper();
	void init_pos();
	pcl::PointCloud<pcl::PointXYZ> detectObjects();
	geometry_msgs::Pose getPoseFromObject();
	geometry_msgs::Pose getPoseFromCoord(double, double, double);
	double* inverseKinematic(geometry_msgs::Pose, double [], int);
	int trajectoryChecker(moveit_msgs::RobotTrajectory);
	void motionPlanner();
	std_msgs::String URScriptCommand(double [], double, double, double, double);
	void cartesianControl();
	void move_arm();
	void shakingMotion();
	void stirringMotion();
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
	ik_client = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");

	group = new moveit::planning_interface::MoveGroupInterface("manipulator");
	// group->setPlannerId("RRTConfigDefault");
	group->setPlannerId("TRRT");
    group->setGoalTolerance(0.01);
	group->setPlanningTime(10);

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

bool ur5Behavior::close_gripper()
{
	robotiq_85_msgs::GripperCmd msg;
	msg.position = 0.0;
	msg.speed = 1.0;
	msg.force = 60.0;
	gripper_pub.publish(msg);

	ros::spinOnce();	
	return 0;
}

bool ur5Behavior::open_gripper()
{	
	robotiq_85_msgs::GripperCmd msg;
	msg.position = 1.0;
	msg.speed = 1.0;
	msg.force = 100.0;
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

void ur5Behavior::move_arm()
{
	ROS_INFO("Moving the Arm.");
	group->move();
	ros::spinOnce();
}

pcl::PointCloud<pcl::PointXYZ> ur5Behavior::detectObjects()
{
	pcl::PointCloud<pcl::PointXYZ> object;
	ros::ServiceClient client = n.serviceClient<bimur_robot_vision::TabletopPerception>("/bimur_object_detector/detect");
	bimur_robot_vision::TabletopPerception srv;
	
	ROS_INFO("Detecting the Objects");
	if(client.call(srv))
	{	
		// Shut Down if the cannot find the plane.
		if(srv.response.is_plane_found == false)
		{
			ROS_ERROR("No object Found. Exitting the Code.");
			ros::shutdown();
		}
		
		int num_objects = srv.response.cloud_clusters.size();
		std::vector<pcl::PointCloud<pcl::PointXYZ>> detected_objects;
		ROS_INFO("Number of Objects Found : %i", num_objects);


		// Convert object to PCL format		
		for (int i = 0; i < num_objects; i++)
		{
			pcl::PointCloud<pcl::PointXYZ> cloud_i;
			pcl::fromROSMsg(srv.response.cloud_clusters[i], cloud_i);
			detected_objects.push_back(cloud_i);
		}


		// Find the largest object out of all.
		int object_index = 0;
		int max = detected_objects[0].points.size();
		for (int i = 0; i < detected_objects.size(); i++)
		{
			int num_points = detected_objects[i].points.size();
			if (num_points > max)
			{
				max = num_points;
				object_index = i;
			}
		}		
		object = detected_objects[object_index];
		target_object = object;
		frame_id = object.header.frame_id;
		ROS_INFO_STREAM(frame_id);
		ros::spinOnce();
	}

	ros::spinOnce();
	return object;
}

geometry_msgs::Pose ur5Behavior::getPoseFromObject()
{
	geometry_msgs::Pose pose_i;
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(target_object, centroid);

	pose_i.position.x = centroid(0);
	pose_i.position.y = centroid(1);
	pose_i.position.z = Z_coord;
	pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.50, 0, 0);

	return pose_i;
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

int ur5Behavior::trajectoryChecker(moveit_msgs::RobotTrajectory trajectoryPlan)
{
	ROS_INFO("Checking the Trajectory");
	std::vector<trajectory_msgs::JointTrajectoryPoint> trajectoryPoints;
	trajectoryPoints = trajectoryPlan.joint_trajectory.points;

	std::vector<int>::size_type vectorSize = trajectoryPoints.size();

	for (int i = 0; i < vectorSize; i++)
	{
		if (trajectoryPoints[i].positions[0] < -1.30173 || trajectoryPoints[i].positions[0] > 0.52330)
		{	
			// ROS_INFO_STREAM(trajectoryPlan);
			ROS_WARN("Trajectory not Feasible due to position 0");
			std::cout<<"Point : "<<i;
			// pressEnter();
			return 0;
		}
		
		if (trajectoryPoints[i].positions[1] < -2.27893 || trajectoryPoints[i].positions[1] > -0.03000)
		{	
			// ROS_INFO_STREAM(trajectoryPlan);
			ROS_WARN("Trajectory not Feasible due to position 1");
			std::cout<<"Point : "<<i;
			// pressEnter();
			return 0;
		}
		
		if (trajectoryPoints[i].positions[2] < 0.90000 || trajectoryPoints[i].positions[2] > 2.77799)
		{	
			// ROS_INFO_STREAM(trajectoryPlan);
			ROS_WARN("Trajectory not Feasible due to position 2");
			std::cout<<"Point : "<<i;
			// pressEnter();
			return 0;
		}

		if (trajectoryPoints[i].positions[3] < 3.50000 || trajectoryPoints[i].positions[3] > 5.00000)
		{	
			// ROS_INFO_STREAM(trajectoryPlan);
			ROS_WARN("Trajectory not Feasible due to position 3");
			std::cout<<"Point : "<<i;
			// pressEnter();
			return 0;
		}

		if (trajectoryPoints[i].positions[4] < -4.60000 || trajectoryPoints[i].positions[4] > -3.80000)
		{	
			// ROS_INFO_STREAM(trajectoryPlan);
			ROS_WARN("Trajectory not Feasible due to position 4");
			std::cout<<"Point : "<<i;
			// pressEnter();
			return 0;
		}

		if (trajectoryPoints[i].positions[5] < -6.67000 || trajectoryPoints[i].positions[5] > 1.370000)
		{	
			// ROS_INFO_STREAM(trajectoryPlan);
			ROS_WARN("Trajectory not Feasible due to position 5");
			std::cout<<"Point : "<<i;
			// pressEnter();
			return 0;
		}
	}
	ros::spinOnce();
	return 1;
}

void ur5Behavior::motionPlanner()
{
	double temp_joint[6];
	geometry_msgs::Pose pose_i = getPoseFromObject();	
	double* joint_angles = inverseKinematic(pose_i, temp_joint, 1);

	int choice = 0;
	int numOfTries = 10;
	while(choice != 1)
	{
		ROS_INFO("Planning the Motion");
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		moveit::planning_interface::MoveItErrorCode planStatus = group->plan(my_plan);
		ROS_INFO("Plan Status : %s", planStatus ? "Success" : "Failed");
		
		if (planStatus)
		{
			moveit_msgs::RobotTrajectory trajectoryPlan = my_plan.trajectory_;
			choice = trajectoryChecker(trajectoryPlan);
			// choice = 1;

			if (choice == 0)
			{
				ROS_WARN("Trajectory Status : NOT GOOD");
				if (numOfTries > 0)
					numOfTries -= 1;
			}
			else
			{
				ROS_INFO("Trajectory Status : Good");
			}
		}

		if (numOfTries == 0 && choice == 0)
		{
			ROS_ERROR("No Feasible Plan possible for the position.");
			ROS_INFO("Exitting the Code.");
			init_pos();
			ros::shutdown();
		}
	}
	ros::spinOnce();
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

void ur5Behavior::cartesianControl()
{
	double x = 0.429;
	double y = -0.278;
	double z = 0.736;
	double Rx = 0;
	double Ry = 0;
	double Rz = 0;

	double a = 0.25;
	double v = 0.25;
	double t = 0;
	double r = 0;


	int trigger = 99;
	while (trigger != 48)
	{
		std::cout<<"Controller : ";
		trigger = getch();
		std::cout<<trigger<<"\n";

		switch (trigger)
		{
			case 105: // i - key
						z -= 0.005;
						break;

			case 107: // k - key
						z += 0.005;
						break;

			case 108: // l - key
						x -= 0.005;
						break;

			case 106: // j - key
						x += 0.005;
						break;

			case 111: // o - key
						y -= 0.005;
						break;

			case 117: // u - key
						y += 0.005;
						break;




			// case 119: // w - key
			// 			Rz -= 0.205;
			// 			break;

			// case 115: // s - key
			// 			Rz += 0.205;
			// 			break;

			// case 100: // d - key
			// 			Rx -= 0.205;
			// 			break;

			// case 97: // a - key
			// 			Rx += 0.205;
			// 			break;

			// case 101: // e - key
			// 			Ry -= 0.205;
			// 			break;

			// case 113: // q - key
			// 			Ry += 0.205;
			// 			break;
		}
		// std::cout<<"\nX : "<<x<<", Y : "<<y<<", Z : "<<z<<"\n";
		double temp_joints[6];
		geometry_msgs::Pose pose_i = getPoseFromCoord(x, y, z);	
		double* joint_angles = inverseKinematic(pose_i, temp_joints);
		std_msgs::String command = URScriptCommand(joint_angles, a, v, t, r); 
		ROS_INFO_STREAM(command);
		command_pub.publish(command);
	}
}

void ur5Behavior::shakingMotion()
{
	double temp_joints[6];
	detectObjects();
	geometry_msgs::Pose pose_i = getPoseFromObject();
	double* joint_angles = inverseKinematic(pose_i, temp_joints);
	ROS_INFO("Moving Above the Object");
	std_msgs::String command = URScriptCommand(joint_angles, 0.90, 0.90);
	command_pub.publish(command);
	getch();

	// Lowering the Arm
	ROS_INFO("Lowering the arm.");
	geometry_msgs::Pose pose_2 = getPoseFromCoord(pose_i.position.x, pose_i.position.y+0.02, pose_i.position.z+0.10);
	joint_angles = inverseKinematic(pose_2, temp_joints);
	command = URScriptCommand(joint_angles, 0.90, 0.90);
	command_pub.publish(command);
	getch();
	close_gripper();
	getch();


	// Going to the action position.
	ROS_INFO("Moving towards the action position.");
	geometry_msgs::Pose pose_3 = getPoseFromCoord(0.04, -0.07, Z_coord);
	joint_angles = inverseKinematic(pose_3, temp_joints);
	command = URScriptCommand(joint_angles, 0.90, 0.90);
	command_pub.publish(command);
	getch();


	// Shake it up!!
	int rotations = 10;
	double a = 1.50;
	double v = 1.50;
	double radius = 0.750;
	ROS_INFO("Executing the Motion.");
	while (rotations > 0)
	{	
		std::cout<<"[ INFO] Rotations : "<<rotations<<"\n";
		joint_angles[3] -= radius;
		joint_angles[4] -= radius;
		command = URScriptCommand(joint_angles, a, v);
		command_pub.publish(command);
		sleep(1.5);


		joint_angles[3] += radius;
		joint_angles[4] += radius;
		command = URScriptCommand(joint_angles, a, v);
		command_pub.publish(command);
		sleep(1.5);


		joint_angles[3] += radius;
		// joint_angles[4] -= radius;
		command = URScriptCommand(joint_angles, a, v);
		command_pub.publish(command);
		sleep(1.5);

		joint_angles[3] -= radius;
		// joint_angles[4] += radius;
		command = URScriptCommand(joint_angles, a, v);
		command_pub.publish(command);
		sleep(1.5);
		rotations--;
	}

	// Going to the action position.
	ROS_INFO("Moving towards the action position.");
	pose_3 = getPoseFromCoord(0.04, -0.07, Z_coord);
	joint_angles = inverseKinematic(pose_3, temp_joints);
	command = URScriptCommand(joint_angles);
	command_pub.publish(command);
	getch();
	open_gripper();
	ros::spinOnce();
}

void ur5Behavior::stirringMotion()
{
	ROS_INFO("Attach the end effector.");
	getch();
	close_gripper();
	getch();

	int rotations = 10;
	double a = 1.50;
	double v = 1.50;
	double radius = 0.05;

	double temp_joints[6];
	detectObjects();
	geometry_msgs::Pose pose_i = getPoseFromObject();
	pose_i.position.z -= 0.10;
	double* joint_angles = inverseKinematic(pose_i, temp_joints);
	ROS_INFO("Moving Above the Container");
	std_msgs::String command = URScriptCommand(joint_angles, 0.90, 0.90);
	command_pub.publish(command);
	getch();

	// Lowering the Arm
	ROS_INFO("Lowering the arm.");
	geometry_msgs::Pose pose_2 = getPoseFromCoord(pose_i.position.x, pose_i.position.y, pose_i.position.z+0.10);
	joint_angles = inverseKinematic(pose_2, temp_joints);
	command = URScriptCommand(joint_angles, 0.90, 0.90);
	command_pub.publish(command);
	getch();

	while (rotations > 0)
	{	
		std::cout<<"[ INFO] Rotations : "<<rotations<<"\n";
		joint_angles[0] -= radius;
		joint_angles[2] += radius;
		// joint_angles[5] += radius;
		std_msgs::String point_2 = URScriptCommand(joint_angles, a, v);
		command_pub.publish(point_2);
		ros::Duration(0.5).sleep();
		
		joint_angles[1] -= radius;
		joint_angles[2] += radius;
		// joint_angles[5] -= radius;
		std_msgs::String point_3 = URScriptCommand(joint_angles, a, v);
		// command_pub.publish(point_3);
		ros::Duration(0.5).sleep();

		joint_angles[0] += radius;
		joint_angles[2] -= radius;
		// joint_angles[5] += radius;
		std_msgs::String point_4 = URScriptCommand(joint_angles, a, v);
		command_pub.publish(point_4);
		ros::Duration(0.5).sleep();

		joint_angles[1] += radius;
		joint_angles[2] -= radius;
		// joint_angles[5] -= radius;
		std_msgs::String point_1 = URScriptCommand(joint_angles, a, v);
		command_pub.publish(point_1);
		ros::Duration(0.5).sleep();

		rotations--;
	}
	open_gripper();

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_arm");

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ur5Behavior Obj;
	Obj.init_pos();
	// Obj.stirringMotion();
	Obj.cartesianControl();
	// Obj.shakingMotion();

	// ros::Duration(2).sleep();
	// double temp_joints[6];
	// geometry_msgs::Pose pose = Obj.getPoseFromCoord(-0.421, 0.443, Obj.Z_coord);
	// double* joint_angles = Obj.inverseKinematic(pose, temp_joints);
	// ROS_INFO_STREAM(joint_angles[0]);
	// std_msgs::String command = Obj.URScriptCommand(joint_angles); 
	// Obj.command_pub.publish(command);
	// Obj.detectObjects();
	// Obj.motionPlanner();
	// Obj.move_arm();
	// Obj.init_pos();
	// Obj.cartesianControl();

	spinner.stop();

	return 0;
}




























// // Declaring the ROS Publisher, Service client, and other variables.
// ros::Publisher pose_pub;
// ros::Publisher gripper_pub;
// ros::Publisher command_pub;
// ros::ServiceClient ik_client;
// geometry_msgs::PoseStamped current_button_pose;
// moveit::planning_interface::MoveGroupInterface *group;
// float Z_OFFSET = 0.25;
// float NEW_Z_OFFSET = 0.0;
// double Z_coord = 0.704;
// bool g_caught_sigint = false;



// /**
//  * @brief 
//  * 
//  * @param sig 
//  */
// void sig_handler(int sig)
// {	
// 	g_caught_sigint = true;
//     ROS_ERROR("Caught sigint, SHUTDOWN...");
//     ros::shutdown();
//     exit(1);
// }

// /**
//  * @brief Function to block the execution temporarily.
//  * 
//  */
// void pressEnter()
// {
// 	std::cout << "************** Press the Any key to continue ************** ";
// 	std::cin.get();
// }


// int getch(void)
// {
//     struct termios oldattr, newattr;
//     int ch;
//     tcgetattr( STDIN_FILENO, &oldattr );
//     newattr = oldattr;
//     newattr.c_lflag &= ~( ICANON | ECHO );
//     tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
//     ch = getchar();
//     tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
//     return ch;
// }


// /**
//  * @brief Function to close the gripper of the ARM.
//  * 
//  * @return true if the execution is successfull.
//  * @return false if the execution is unsuccessfull.
//  */
// bool close_gripper()
// {
// 	robotiq_85_msgs::GripperCmd msg;
// 	msg.position = 0.0;
// 	msg.speed = 1.0;
// 	msg.force = 100.0;
// 	gripper_pub.publish(msg);

// 	ros::spinOnce();	
// 	return 0;
// }


// /**
//  * @brief Function to open the gripper of the ARM.
//  * 
//  * @return true if the execution is successfull.
//  * @return false if the execution is unsuccessfull.
//  */
// bool open_gripper()
// {	
// 	robotiq_85_msgs::GripperCmd msg;
// 	msg.position = 1.0;
// 	msg.speed = 1.0;
// 	msg.force = 100.0;
// 	gripper_pub.publish(msg);

// 	ros::spinOnce();
// 	return 0;
// }


// int trajectoryChecker(moveit_msgs::RobotTrajectory trajectoryPlan)
// {
// 	/* There are 6 joints in the arm which are as follows:
// 	1. Base
// 	2. Shoulder
// 	3. Elbow
// 	4. Wrist 1 
// 	5. Wrist 2
// 	6. Wrist 3

// 	Now, All the improper trajectory have something in common, all of them have crosses limit of BASE, SHOULDER and ELBOW joint. So, if we figure out the 
// 	way to determine that the plan is crossing movement_threshold of the BASE, SHOULDER and ELBOW joint then we can determine if the plan is proper or not.
// 	*/

// 	ROS_INFO("Checking the Trajectory");
// 	std::vector<trajectory_msgs::JointTrajectoryPoint> trajectoryPoints;
// 	trajectoryPoints = trajectoryPlan.joint_trajectory.points;

// 	std::vector<int>::size_type vectorSize = trajectoryPoints.size();
// 	// double w1_min, w2_min, w3_min, w1_max, w2_max, w3_max;

// 	for (int i = 0; i < vectorSize; i++)
// 	{
// 		if (trajectoryPoints[i].positions[0] < -1.30173 || trajectoryPoints[i].positions[0] > 0.52330)
// 		{	
// 			ROS_INFO_STREAM(trajectoryPlan);
// 			ROS_WARN("Trajectory not Feasible due to position 0");
// 			std::cout<<"Point : "<<i;
// 			// pressEnter();
// 			return 0;
// 		}
		
// 		if (trajectoryPoints[i].positions[1] < -2.27893 || trajectoryPoints[i].positions[1] > -0.03000)
// 		{	
// 			ROS_INFO_STREAM(trajectoryPlan);
// 			ROS_WARN("Trajectory not Feasible due to position 1");
// 			std::cout<<"Point : "<<i;
// 			// pressEnter();
// 			return 0;
// 		}
		
// 		if (trajectoryPoints[i].positions[2] < 0.90000 || trajectoryPoints[i].positions[2] > 2.77799)
// 		{	
// 			ROS_INFO_STREAM(trajectoryPlan);
// 			ROS_WARN("Trajectory not Feasible due to position 2");
// 			std::cout<<"Point : "<<i;
// 			// pressEnter();
// 			return 0;
// 		}

// 		if (trajectoryPoints[i].positions[3] < 3.50000 || trajectoryPoints[i].positions[3] > 5.00000)
// 		{	
// 			ROS_INFO_STREAM(trajectoryPlan);
// 			ROS_WARN("Trajectory not Feasible due to position 3");
// 			std::cout<<"Point : "<<i;
// 			// pressEnter();
// 			return 0;
// 		}

// 		if (trajectoryPoints[i].positions[4] < -4.60000 || trajectoryPoints[i].positions[4] > -3.80000)
// 		{	
// 			ROS_INFO_STREAM(trajectoryPlan);
// 			ROS_WARN("Trajectory not Feasible due to position 4");
// 			std::cout<<"Point : "<<i;
// 			// pressEnter();
// 			return 0;
// 		}

// 		if (trajectoryPoints[i].positions[5] < -6.67000 || trajectoryPoints[i].positions[5] > 1.370000)
// 		{	
// 			ROS_INFO_STREAM(trajectoryPlan);
// 			ROS_WARN("Trajectory not Feasible due to position 5");
// 			std::cout<<"Point : "<<i;
// 			// pressEnter();
// 			return 0;
// 		}
// 		// if (i == 0)
// 		// {
// 		// 	w1_min = trajectoryPoints[i].positions[3];
// 		// 	w1_max = trajectoryPoints[i].positions[3];

// 		// 	w2_min = trajectoryPoints[i].positions[4];
// 		// 	w2_max = trajectoryPoints[i].positions[4];

// 		// 	w3_min = trajectoryPoints[i].positions[5];
// 		// 	w3_max = trajectoryPoints[i].positions[5];
// 		// }
// 		// else
// 		// {
// 		// 	w1_min = std::min(w1_min, trajectoryPoints[i].positions[3]);
// 		// 	w1_max = std::max(w1_max, trajectoryPoints[i].positions[3]);

// 		// 	w2_min = std::min(w2_min, trajectoryPoints[i].positions[4]);
// 		// 	w2_max = std::max(w2_max, trajectoryPoints[i].positions[4]);

// 		// 	w3_min = std::min(w3_min, trajectoryPoints[i].positions[5]);
// 		// 	w3_max = std::max(w3_max, trajectoryPoints[i].positions[5]);
// 		// }
// 	}
// 	// ROS_WARN("W1_min : %f", w1_min);
// 	// ROS_WARN("W1_max : %f", w1_max);
// 	// ROS_WARN("W2_min : %f", w2_min);
// 	// ROS_WARN("W2_max : %f", w2_max);
// 	// ROS_WARN("W3_min : %f", w3_min);
// 	// ROS_WARN("W3_max : %f", w3_max);
// 	return 1;
// }


// /**
//  * @brief Function to move the arm in the initial position using joint angles. The values MUST match
// 	or it will not be moving to the correct hard coded position.
//  * 
//  */
// void init_pos()
// {
// 	std::map<std::string, double> target;
// 	target["shoulder_pan_joint"] =  0.4166615605354309;
// 	target["shoulder_lift_joint"] =  -1.6050642172442835;
// 	target["elbow_joint"] =  2.164191246032715;
// 	target["wrist_1_joint"] = 3.720980167388916;
// 	target["wrist_2_joint"] = -4.021045986806051;
// 	target["wrist_3_joint"] = -0.6397879759417933;
	
// 	group->setJointValueTarget(target);
// 	group->move();

// 	// ROS_INFO_STREAM(group->getCurrentPose().pose.position.x);
// }


// /**
//  * @brief Plan the motion to move the ARM.
//  * 
//  */
// void motionPlanner(pcl::PointCloud<pcl::PointXYZ> target_object, double x, double y, double z = Z_coord)
// {
// 	geometry_msgs::Pose pose_i;
// 	geometry_msgs::PoseStamped stampedPose, stampOut;
// 	tf::TransformListener listener;
// 	moveit_msgs::GetPositionIK::Request ik_request;
//     moveit_msgs::GetPositionIK::Response ik_response;


// 	pose_i.position.x = x;
// 	pose_i.position.y = y;
// 	pose_i.position.z = z;
// 	pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
	
// 	stampedPose.header.frame_id = target_object.header.frame_id;
// 	stampedPose.header.stamp = ros::Time(0);
// 	stampedPose.pose = pose_i;
 
// 	listener.waitForTransform(stampedPose.header.frame_id, "world", ros::Time(0), ros::Duration(3.0));
// 	listener.transformPose("world", stampedPose, stampOut);

// 	stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14/2, 3.14/2, 0.0);
// 	stampOut.pose.position.z += Z_OFFSET;

// 	ROS_INFO("Publishing the Pose");
// 	pose_pub.publish(stampOut);

// 	group->setPoseReferenceFrame(stampOut.header.frame_id);
// 	group->setPoseTarget(stampOut);
//     group->setStartState(*group->getCurrentState());
    
// 	ik_request.ik_request.group_name = "manipulator";
//     ik_request.ik_request.pose_stamped = stampOut;
//     ik_client.call(ik_request, ik_response);


// 	int choice = 0;
// 	int numOfTries = 10;

// 	while(choice != 1)
// 	{
// 		ROS_INFO("Motion Planning");
// 		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
// 		moveit::planning_interface::MoveItErrorCode planStatus = group->plan(my_plan);
// 		ROS_INFO("Plan Status : %s", planStatus ? "Success" : "Failed");
		
// 		if (planStatus)
// 		{
// 			moveit_msgs::RobotTrajectory trajectoryPlan = my_plan.trajectory_;
// 			choice = trajectoryChecker(trajectoryPlan);

// 			if (choice == 0)
// 			{
// 				if (numOfTries > 0)
// 					numOfTries -= 1;
// 			}
// 		}

// 		if (numOfTries == 0 && choice == 0)
// 		{
// 			ROS_ERROR("No Feasible Plan possible for the position.");
// 			ROS_INFO("Exitting the Code.");
// 			init_pos();
// 			exit(0);
// 		}
// 	}
// 	ros::Duration(1).sleep();
// }


// /**
//  * @brief Function to subscribe to the /bimur_object_detector/detect topic to get cloud 
//  		  clusters by calling the tabletop perception service and figure out how many ob
// 		  -jects are on the table and find the pose of the "largest" found object.
//  * 
//  * @param n ros::NodeHandle (Get Data from the camera)
//  * @return pcl::PointCloud<pcl::PointXYZ> 
//  */
// pcl::PointCloud<pcl::PointXYZ> detectObjects(ros::NodeHandle n)
// {
// 	// Call the button detection service and get the response
// 	pcl::PointCloud<pcl::PointXYZ> target_object;
// 	ros::ServiceClient client = n.serviceClient<bimur_robot_vision::TabletopPerception>("/bimur_object_detector/detect");
// 	bimur_robot_vision::TabletopPerception srv;
	
// 	ROS_INFO("Detecting the Objects");
// 	if(client.call(srv))
// 	{	
// 		// Shut Down if the cannot find the plane.
// 		if(srv.response.is_plane_found == false)
// 		{
// 			ROS_ERROR("No object Found. Exitting the Code.");
// 			ros::shutdown();
// 		}
		
// 		int num_objects = srv.response.cloud_clusters.size();
// 		std::vector<pcl::PointCloud<pcl::PointXYZ>> detected_objects;
// 		ROS_INFO("Number of Objects Found : %i", num_objects);


// 		// Convert object to PCL format		
// 		for (int i = 0; i < num_objects; i++)
// 		{
// 			pcl::PointCloud<pcl::PointXYZ> cloud_i;
// 			pcl::fromROSMsg(srv.response.cloud_clusters[i], cloud_i);
// 			detected_objects.push_back(cloud_i);
// 		}


// 		// Find the largest object out of all.
// 		int target_object_index = 0;
// 		int max = detected_objects[0].points.size();
// 		for (int i = 0; i < detected_objects.size(); i++)
// 		{
// 			int num_points = detected_objects[i].points.size();
// 			if (num_points > max)
// 			{
// 				max = num_points;
// 				target_object_index = i;
// 			}
// 		}		
// 		target_object = detected_objects[target_object_index];

// 		ros::spinOnce();
// 	}
// 	return target_object;
// }


// /**
//  * @brief Function to move the robot above the object.
//  * 
//  */
// void moveAboveObject(pcl::PointCloud<pcl::PointXYZ> target_object)
// {	
// 	ROS_INFO("Starting to move above object");

// 	Eigen::Vector4f centroid;
// 	pcl::compute3DCentroid(target_object, centroid);
// 	ROS_INFO("Object Centroid x: %f", centroid(0));
// 	ROS_INFO("Object Centroid y: %f", centroid(1));
// 	ROS_INFO("Object Centroid z: %f", centroid(2));
    
// 	motionPlanner(target_object, centroid(0), centroid(1), Z_coord-0.10);
//     moveit::planning_interface::MoveItErrorCode error = group->move();
// }


// /**
//  * @brief Function to lower the robot arm to the object.
//  * 
//  */
// void moveToObj(pcl::PointCloud<pcl::PointXYZ> target_object)
// {
// 	ROS_INFO("Lowering the ARM.");

// 	Eigen::Vector4f centroid;
// 	pcl::compute3DCentroid(target_object, centroid);
// 	ROS_INFO("Object Centroid x: %f", centroid(0));
// 	ROS_INFO("Object Centroid y: %f", centroid(1));
// 	ROS_INFO("Object Centroid z: %f", centroid(2));

// 	motionPlanner(target_object, centroid(0), centroid(1), Z_coord+0.10);
//     moveit::planning_interface::MoveItErrorCode error = group->move();
// }


// /**
//  * @brief Function to generate random float in the given range.
//  * 
//  * @param min Lower end of the range
//  * @param max Upper end of the range
//  * @return float
//  */
// float RandomFloat(float min, float max)
// {
// 	srand(time(0));
//     float r = (float)rand() / (float)RAND_MAX;
//     return min + r * (max - min);
// }


// /**
//  * @brief Function to move the object to any random position on the plane.
//  * 
//  * @param target_object 
//  */
// void moveToPos(pcl::PointCloud<pcl::PointXYZ> target_object)
// {
// 	// Defining the random target position.
// 	double x_dest = RandomFloat(0.049, 0.369); // RandomFloat(-0.270, 0.369);
// 	double y_dest = RandomFloat(-0.236, -0.060); // RandomFloat(-0.236, -0.060);

// 	motionPlanner(target_object, x_dest, y_dest);
// 	moveit::planning_interface::MoveItErrorCode error = group->move();

// 	// ROS_INFO_STREAM(ik_response.solution.joint_state.position[0]);
// 	// std::cout<<"\n\n"<<ik_request.ik_request.ik_seed_state.joint_state.name.push_back("shoulder_pan_joint");
// 	// ROS_INFO(ik_response.solution.joint_state.position[0]);
	
// 	// ROS_INFO("Starting the experiment");
// 	// std::map<std::string, double> target;
// 	// target["shoulder_pan_joint"] =  ik_response.solution.joint_state.position[0];
// 	// target["shoulder_lift_joint"] =  ik_response.solution.joint_state.position[1];
// 	// target["elbow_joint"] =  ik_response.solution.joint_state.position[2];
// 	// target["wrist_1_joint"] = ik_response.solution.joint_state.position[3];
// 	// target["wrist_2_joint"] = ik_response.solution.joint_state.position[4];
// 	// target["wrist_3_joint"] = ik_response.solution.joint_state.position[5];
	
// 	// group->setJointValueTarget(target);
// 	// moveit::planning_interface::MoveItErrorCode error = group->move();
// 	// ROS_INFO("End of experiment");



// 	// Lowering the ARM to drop the object.
// 	ROS_INFO("Lowering the ARM.");
// 	motionPlanner(target_object, x_dest, y_dest, Z_coord+0.10);
// 	error = group->move();


// 	ros::Duration(1).sleep();
// 	open_gripper();


// 	// Moving up the arm after dropping the object.
//     ROS_INFO("Moving up the ARM");
// 	motionPlanner(target_object, x_dest, y_dest, Z_coord-0.10);
// 	error = group->move();
// }


// // void socketConnection(char[] IP_ADDR, int PORT, )
// // {
// // 	int sock = 0, valread, client_fd;
// // 	struct sockaddr_in serv_addr;
	
// //     char* hello = "Hello from client \n";
	
// //     char buffer[1024] = { 0 };

// // 	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
// // 	{
// // 		ROS_ERROR("Socket Creation Error");
// // 		return -1;
// // 	}

// // 	serv_addr.sin_family = AF_INET;
// // 	serv_addr.sin_port = htons(PORT);

// // 	if (inet_pton(AF_INET, "172.22.22.2", &serv_addr.sin_addr) <= 0) 
// //     {
// // 		printf("\nInvalid address/ Address not supported \n");
// // 		return -1;
// // 	}

// // 	if ((client_fd = connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr))) < 0)
// //     {
// // 		printf("\nConnection Failed \n");
// // 		return -1;
// // 	}
	
// //     send(sock, hello, strlen(hello), 0);
	
// //     valread = read(sock, buffer, 1024);
// // 	ROS_INFO_STREAM("Message Received : %s", buffer);

// // 	close(client_fd);
// // }


// std_msgs::String URScriptCommand(double joint[], double a = 0.20, double v = 0.20, double t = 0, double r = 0)
// {
// 	std_msgs::String command; 
// 	command.data = "movej([";

// 	command.data += std::to_string(joint[0]);
// 	command.data += ",";

// 	command.data += std::to_string(joint[1]);
// 	command.data += ",";

// 	command.data += std::to_string(joint[2]);
// 	command.data += ",";

// 	command.data += std::to_string(joint[3]);
// 	command.data += ",";

// 	command.data += std::to_string(joint[4]);
// 	command.data += ",";

// 	command.data += std::to_string(joint[5]);
// 	command.data += "],a=";
	
// 	command.data += std::to_string(a);
// 	command.data += ",v=";

// 	command.data += std::to_string(v);
// 	command.data += ",t=";

// 	command.data += std::to_string(t);
// 	command.data += ",r=";
	
// 	command.data += std::to_string(r);
// 	command.data += ")\n";

// 	return command;
// }


// double* inverseKinematic(pcl::PointCloud<pcl::PointXYZ> target_object, double joint[6])
// {
// 	Eigen::Vector4f centroid;
// 	pcl::compute3DCentroid(target_object, centroid);

// 	geometry_msgs::Pose pose_i;
// 	geometry_msgs::PoseStamped stampedPose, stampOut;
// 	tf::TransformListener listener;
// 	moveit_msgs::GetPositionIK::Request ik_request;
//     moveit_msgs::GetPositionIK::Response ik_response;

// 	pose_i.position.x = centroid(0);
// 	pose_i.position.y = centroid(1);
// 	pose_i.position.z = Z_coord;
// 	pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
	
// 	stampedPose.header.frame_id = target_object.header.frame_id;
// 	stampedPose.header.stamp = ros::Time(0);
// 	stampedPose.pose = pose_i;
 
// 	listener.waitForTransform(stampedPose.header.frame_id, "world", ros::Time(0), ros::Duration(3.0));
// 	listener.transformPose("world", stampedPose, stampOut);

// 	stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14/2, 3.14/2, 0.0);
// 	stampOut.pose.position.z += Z_OFFSET;
    
// 	ik_request.ik_request.group_name = "manipulator";
//     ik_request.ik_request.pose_stamped = stampOut;
//     ik_client.call(ik_request, ik_response);


// 	joint[0] = ik_response.solution.joint_state.position[0];
// 	joint[1] = ik_response.solution.joint_state.position[1];
// 	joint[2] = ik_response.solution.joint_state.position[2];
// 	joint[3] = ik_response.solution.joint_state.position[3];
// 	joint[4] = ik_response.solution.joint_state.position[4];
// 	joint[5] = ik_response.solution.joint_state.position[5];

// 	return joint;
// }


// // void basicStir(pcl::PointCloud<pcl::PointXYZ> target_object, double r = 0.10, int rotations = 30)
// // {

// // 	// // double init_x = -0.317;
// // 	// // double init_y = -0.066;
// // 	// // double init_z = 0.541;
// // 	// // double init_Rx = 1.644;
// // 	// // double init_Ry = 1.588;
// // 	// // double init_Rz = 3.895;

// // 	double joint_temp[6];
// // 	double* joint = inverseKinematic(target_object, joint_temp);

// // 	std_msgs::String init_point = URScriptCommand(joint[1], joint[2], joint[3], joint[4], joint[5], joint[6]);
// // 	command_pub.publish(init_point);
// // 	ros::Duration(1.5).sleep();

// // 	while (rotations > 0)
// // 	{	
// // 		std::cout<<"[INFO ]. Rotations : "<<rotations<<"\n";
// // 		std_msgs::String point_2 = URScriptCommand(joint[1] - 0.05, joint[2], joint[3], joint[4], joint[5], joint[6]);
// // 		command_pub.publish(point_2);
// // 		ros::Duration(1.5).sleep();

// // 		std_msgs::String point_3 = URScriptCommand(joint[1], joint[2] - 0.050, joint[3] + 0.050, joint[4], joint[5], joint[6]);
// // 		command_pub.publish(point_3);
// // 		ros::Duration(1.5).sleep();

// // 		std_msgs::String point_4 = URScriptCommand(joint[1] + 0.050, joint[2], joint[3], joint[4], joint[5], joint[6]);
// // 		command_pub.publish(point_4);
// // 		ros::Duration(1.5).sleep();

// // 		std_msgs::String point_1 = URScriptCommand(joint[1], joint[2] + 0.050, joint[3] - 0.050, joint[4], joint[5], joint[6]);
// // 		command_pub.publish(point_1);
// // 		ros::Duration(1.5).sleep();

// // 		rotations--;
// // 	}
// // }


// void cartesianControl(pcl::PointCloud<pcl::PointXYZ> target_object)
// {
// 	double x = -0.277;
// 	double y = -0.099;
// 	double z = 0.552;

// 	double a = 0.25;
// 	double v = 0.25;
// 	double r = 0;


// 	int trigger = 99;
// 	while (trigger != 48)
// 	{
// 		std::cout<<"Controller : ";
// 		trigger = getch();
// 		std::cout<<trigger<<"\n";

// 		switch (trigger)
// 		{
// 			case 107: // k - key
// 						y += 0.005;
// 						break;

// 			case 105: // i - key
// 						y -= 0.005;
// 						break;

// 			case 106: // j - key
// 						x += 0.005;
// 						break;

// 			case 108: // l - key
// 						x -= 0.005;
// 						break;

// 			case 111: // o - key
// 						z += 0.005;
// 						y -= 0.005;
// 						break;

// 			case 117: // u - key
// 						z -= 0.005;
// 						y += 0.005;
// 						break;

// 		}


// 		// Eigen::Vector4f centroid;
// 		// pcl::compute3DCentroid(target_object, centroid);

// 		// geometry_msgs::Pose pose_i;
// 		// geometry_msgs::PoseStamped stampedPose, stampOut;
// 		// tf::TransformListener listener;
// 		// moveit_msgs::GetPositionIK::Request ik_request;
// 		// moveit_msgs::GetPositionIK::Response ik_response;

// 		// pose_i.position.x = x;
// 		// pose_i.position.y = y;
// 		// pose_i.position.z = z;
// 		// pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
		
// 		// stampedPose.header.frame_id = target_object.header.frame_id;
// 		// stampedPose.header.stamp = ros::Time(0);
// 		// stampedPose.pose = pose_i;
	
// 		// listener.waitForTransform(stampedPose.header.frame_id, "world", ros::Time(0), ros::Duration(3.0));
// 		// listener.transformPose("world", stampedPose, stampOut);

// 		// stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14/2, 3.14/2, 0.0);
// 		// stampOut.pose.position.z += Z_OFFSET;
		
// 		// ik_request.ik_request.group_name = "manipulator";
// 		// ik_request.ik_request.pose_stamped = stampOut;
// 		// ik_client.call(ik_request, ik_response);


// 		// double j1 = ik_response.solution.joint_state.position[0];
// 		// double j2 = ik_response.solution.joint_state.position[1];
// 		// double j3 = ik_response.solution.joint_state.position[2];
// 		// double j4 = ik_response.solution.joint_state.position[3];
// 		// double j5 = ik_response.solution.joint_state.position[4];
// 		// double j6 = ik_response.solution.joint_state.position[5];

// 		double joint_temp[6];
// 		double* joint = inverseKinematic(target_object, joint_temp);

// 		std_msgs::String command = URScriptCommand(x, y, z); 
// 		std::cout<<command.data<<"\n";

// 		command_pub.publish(command);
// 	}
// }


// // void cartesianControl_linear()
// // {
// // 	double x = -0.317;
// // 	double y = -0.066;
// // 	double z = 0.541;
// // 	double Rx = 1.644;
// // 	double Ry = 1.588;
// // 	double Rz = 3.895;

// // 	double a = 0.25;
// // 	double v = 0.25;
// // 	double r = 0;


// // 	int trigger = 99;
// // 	while (trigger != 48)
// // 	{
// // 		// char buffer[1024] = { 0 };

// // 		std::cout<<"Controller : ";
// // 		trigger = getch();
// // 		std::cout<<trigger<<"\n";

// // 		switch (trigger)
// // 		{
// // 			case 107: // k - key
// // 						y += 0.050;
// // 						break;

// // 			case 105: // i - key
// // 						y -= 0.050;
// // 						break;

// // 			case 106: // j - key
// // 						x += 0.050;
// // 						break;

// // 			case 108: // l - key
// // 						x -= 0.050;
// // 						break;

// // 			case 111: // o - key
// // 						z += 0.050;
// // 						y -= 0.050;
// // 						break;

// // 			case 117: // u - key
// // 						z -= 0.050;
// // 						y += 0.050;
// // 						break;

// // 			case 115: // s - key
// // 						Ry += 0.050;
// // 						break;

// // 			case 119: // w - key
// // 						Ry -= 0.050;
// // 						break;
			
// // 			case 100: // d - key
// // 						Rx += 0.200;
// // 						break;
			
// // 			case 97: // a - key
// // 						Rx -= 0.200;
// // 						break;
			
// // 			case 113: // q - key
// // 						Rz += 0.200;
// // 						Ry += 0.050;
// // 						break;
			
// // 			case 101: // e - key
// // 						Rz -= 0.200;
// // 						Ry -= 0.050;
// // 						break;
// // 		}

// // 		std_msgs::String command = URScriptCommand(x, y, z, Rx, Ry, Rz); 
// // 		std::cout<<command.data<<"\n";

// // 		command_pub.publish(command);
// // 	}
// // }



// /**
//  * @brief Main Driver code. (Known optimal planners: RRT, PRM)
//  * 
//  * @param argc 
//  * @param argv 
//  * @return int 
//  */
// int main(int argc, char **argv)
// {	

// 	ros::init(argc, argv, "move_arm");

// 	ur5Behavior Obj;














// 	// Intialize ROS with this node name
// 	ros::init(argc, argv, "move_arm");
// 	ros::NodeHandle n;
// 	ros::AsyncSpinner spinner(1);
//     spinner.start();
    
// 	ik_client = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
// 	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_arm_demo/pose", 10);
// 	gripper_pub = n.advertise<robotiq_85_msgs::GripperCmd>("/gripper/cmd", 10);
// 	command_pub = n.advertise<std_msgs::String>("/ur_hardware_interface/script_command", 10);

// 	ROS_INFO("ARM Movement DEMO");

// 	//Need to only call MoveGroupInterface ONCE in the program to allow the ur5 to accept targets, and move to them
// 	group = new moveit::planning_interface::MoveGroupInterface("manipulator");
// 	// group->setPlannerId("RRTConfigDefault");
// 	group->setPlannerId("TRRT");
//     group->setGoalTolerance(0.01);
// 	group->setPlanningTime(10);

// 	int temp = 1;
// 	while(temp > 0)
// 	{
// 		// Initialize the ARM and open the gripper
// 		open_gripper();
// 		init_pos();
// 		ros::Duration(2).sleep();
		
// 		// Detecting the objects and there positions
// 		pcl::PointCloud<pcl::PointXYZ> target_object = detectObjects(n);
// 		ros::Duration(2).sleep();
// 		// pressEnter();

// 		Eigen::Vector4f centroid;
// 		pcl::compute3DCentroid(target_object, centroid);
		
// 		// basicStir(target_object);
// 		// ros::Duration(2).sleep();

// 		cartesianControl(target_object);
// 		ros::Duration(2).sleep();

// 		// close_gripper();
// 		// ros::Duration(2).sleep();

// 		// init_pos();
// 		// ros::Duration(2).sleep();

// 		// open_gripper();
// 		// ros::Duration(2).sleep();

// 		// init_pos();
// 		// ros::Duration(2).sleep();









// 		// // Move ARM over the object.
// 		// ROS_INFO("\n\n\n\n\n\n Moving Above Object");
// 		// moveAboveObject(target_object);
// 		// ros::Duration(2).sleep();

// 		// // Move down the Position to pick the object.
// 		// ROS_INFO("\n\n\n\n\n\n Moving To Object");
// 		// moveToObj(target_object);
// 		// ros::Duration(2).sleep();

// 		// // Close the gripper
// 		// close_gripper();
// 		// ros::Duration(2).sleep();
// 		// // pressEnter();

// 		// // Go to initial position
// 		// init_pos();
// 		// ros::Duration(2).sleep();
// 		// // pressEnter();

// 		// // Compute a random position on the surface and move the ARM.
// 		// ROS_INFO("\n\n\n\n\n\n Moving To Target");
// 		// moveToPos(target_object);
// 		// ros::Duration(2).sleep();
// 		// // pressEnter();

// 		// // Go to initial position.
// 		// init_pos();

// 		temp -= 1;
// 	}

// 	return 0;
// }
