
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <string>
#include <fstream>
#include <iostream> 
#include <cstdlib>
#include "joint_recorder/recorderMsg.h"
#include <chrono>
#include <ctime>

#include <sstream>
#include <librealsense2/rs.hpp>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#include <stdlib.h>
#include <string.h>
#include <iomanip>

bool recordFlag = false;
std::ofstream file; 
std::string folderName, thisNodeName;
int num1 = 0;

void frame_topic1_callback(const sensor_msgs::ImageConstPtr& msg) {
    if (recordFlag) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
          ROS_WARN("INSIDE topic1 callback");
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        std::ostringstream count;
        count << std::internal << std::setfill('0') << std::setw(5) << num1;

        ros::Time timestamp = msg->header.stamp;
        double timestamp2 = std::stod(std::to_string(timestamp.sec) + "." + std::to_string(timestamp.nsec));

        std::stringstream stream;
        stream << std::fixed << std::setprecision(9) << timestamp2;
        std::string timestamp3 = stream.str();
        
        std::string img_file = folderName + count.str() + "_" + timestamp3.c_str() + ".jpg";

        cv::imwrite(img_file, cv_ptr->image);

        num1++;
    }
    ros::spinOnce();
}

void recordingControlCallback(const joint_recorder::recorderMsg::ConstPtr& msg) {
    if ((msg->command.data.compare("set_file_name") == 0) && (msg->topic.data.compare(thisNodeName) == 0)) {
      ROS_INFO("Setted the File name.");
      folderName = msg->fileName.data;
    }    
    else if (!folderName.empty()) { 
      if (msg->command.data.compare("start") == 0) { 
        ROS_INFO("In Start condition of recordService callback function");
        recordFlag = true;
        num1 = 0;
      }
      else if (msg->command.data.compare("stop") == 0) {
          ROS_ERROR("In STOP condition of recordService callback function"); //Stop recording but keep node running
          recordFlag = false;
          num1 = 0;
          folderName = "";
      }
      else if (msg->command.data.compare("shutdown") == 0) { // shut node down
          ROS_ERROR("Shutting down");
          ros::shutdown();
      }      
      else {
        ROS_ERROR("Command Should be \"set_file_name\" \"start\" or \"stop\" or or \"shutdown\"");
      }
    }
    else ("File name needs to be set before sending command to start recording");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "color_frame_recorder_node");
  thisNodeName = "color_frame_capture";

  ros::NodeHandle n;

  ros::Subscriber controlSub = n.subscribe("recording_control_topic", 10, recordingControlCallback);
  ros::Subscriber color_sub = n.subscribe("/camera/color/image_raw", 10, &frame_topic1_callback);

  while (ros::ok()) {
    // ROS_INFO("Listening to /camera/color/image_raw");
    ros::spinOnce();
  }
  return 0;
}
