
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

#include<stdlib.h>
#include<string.h>


bool recordFlag = false;
std::ofstream file; 
std::string folderName, thisNodeName;
int num1 = 1;
int num2 = 1;


void frame_topic1_callback(const sensor_msgs::ImageConstPtr& msg)
{
    if (recordFlag)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_WARN("INSIDE topic1 callback");
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        std::string png_file = folderName + "image-" + std::to_string(num1) + ".jpg";  
        cv::imwrite(png_file, cv_ptr->image);
        
        std::string command = "python3 /home/mulip-admin/bimur_ws/src/UR5-ros-melodic/digit/src/recordTouchData.py --num " + std::to_string(num2);
        system(command.c_str());

        num1++;
        num2++;
    }
    ros::spinOnce(); 
}

void recordingControlCallback(const joint_recorder::recorderMsg::ConstPtr& msg)
{
    if((msg->command.data.compare("set_file_name") == 0) && (msg->topic.data.compare(thisNodeName) == 0))
    {
      ROS_INFO("Setted the File name.");
      folderName = msg->fileName.data;
    }
    
    else if(!folderName.empty())
    { 
      if (msg->command.data.compare("start") == 0)
      { 
        ROS_INFO("In Start condition of recordService callback function");
        recordFlag = true;
      }
      
      else if (msg->command.data.compare("stop") == 0)
      {
          ROS_ERROR("In STOP condition of recordService callback function"); //Stop recording but keep node running
          recordFlag = false;
          num1 = 1;
          num2 = 1;
          folderName = "";

      }
      
      else if (msg->command.data.compare("shutdown") == 0) //shut node down
      {
          ROS_ERROR("Shutting down");
          ros::shutdown();
      }
      
      else
      {
        ROS_ERROR("Command Should be \"set_file_name\" \"start\" or \"stop\" or or \"shutdown\"");
      }
    }

    else("File name needs to be set before sending command to start recording");
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_frame_recorder_node");
  thisNodeName = "color_frame_capture";

  ros::NodeHandle n;

  ros::Subscriber controlSub = n.subscribe("recording_control_topic", 10, recordingControlCallback);
  ros::Subscriber color_sub = n.subscribe("/camera/color/image_raw", 10, &frame_topic1_callback);

  while(ros::ok())
  {
    ROS_INFO("Listening to /camera/color/image_raw");
    ros::spinOnce();
  }
  return 0;
}
























// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include <string>
// #include <fstream>
// #include <iostream> 
// #include <cstdlib>
// #include "joint_recorder/recorderMsg.h"
// #include <chrono>
// #include <ctime>

// #include <sstream>
// #include <librealsense2/rs.hpp>

// #define STB_IMAGE_WRITE_IMPLEMENTATION
// #include "stb_image_write.h"


// bool recordFlag = false;
// std::ofstream file; 
// std::string folderName, thisNodeName;
// int num = 1;

// void recordingControlCallback(const joint_recorder::recorderMsg::ConstPtr& msg)
// {
//     // std::cout<<"\n"<<msg->command.data<<"\n"<<msg->topic.data<<" | "<<thisNodeName<<"\n";
//     ROS_INFO("Callback Function");
    
//     // Declare depth colorizer for pretty visualization of depth data
//     rs2::colorizer color_map;
    
//     // Declare RealSense pipeline, encapsulating the actual device and sensors
//     rs2::pipeline pipe;
    
//     // Start streaming with default recommended configuration
//     pipe.start();

//     if((msg->command.data.compare("set_file_name") == 0) && (msg->topic.data.compare(thisNodeName) == 0))
//     {
//       ROS_INFO("Setted the File name.");
//       folderName = msg->fileName.data;

//       // Capture 30 frames to give autoexposure, etc. a chance to settle
//       // for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();
//     }
    
//     else if(!folderName.empty())
//     { 
//       if (msg->command.data.compare("start") == 0)
//       {
//         ROS_INFO("In Start condition of recordService callback function");
//         recordFlag = true;

//         while (recordFlag && ros::ok())
//         { 
//             ROS_INFO("Inside While loop. Starting the recording.");
//             // // Declare depth colorizer for pretty visualization of depth data
//             // rs2::colorizer color_map;
            
//             // // Declare RealSense pipeline, encapsulating the actual device and sensors
//             // rs2::pipeline pipe;
            
//             // // Start streaming with default recommended configuration
//             // pipe.start();

//             // Capture 30 frames to give autoexposure, etc. a chance to settle
//             for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();

//             // Wait for the next set of frames from the camera. Now that autoexposure, etc.
//             // has settled, we will write these to disk
//             for (auto&& frame : pipe.wait_for_frames())
//             {
//                 // We can only save video frames as pngs, so we skip the rest
//                 if (auto vf = frame.as<rs2::video_frame>())
//                 {
//                     auto stream = frame.get_profile().stream_type();
//                     // Use the colorizer to get an rgb image for the depth stream
//                     if (vf.is<rs2::depth_frame>()) vf = color_map.process(frame);
                    

//                     std::stringstream png_file;
                    
//                     png_file << folderName << "image - " << vf.get_profile().stream_name() << std::to_string(num) << ".jpg";
                    
//                     stbi_write_jpg(png_file.str().c_str(), vf.get_width(), vf.get_height(), vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
                    
//                     ROS_INFO("Saved : %s", png_file.str());
//                     num++;
//                 }
//             }
//             ros::spinOnce();
//           }     
//           ROS_ERROR("End of start condition. IT SHOULD NOT BE RECORDING HERE.");
//       }
      
//       else if (msg->command.data.compare("stop") == 0)
//       {
//           ROS_ERROR("In STOP condition of recordService callback function"); //Stop recording but keep node running
//           recordFlag = false;
//       }
      
//       else if (msg->command.data.compare("shutdown") == 0) //shut node down
//       {
//           ROS_ERROR("Shutting down");
//           ros::shutdown();
//       }
      
//       else
//       {
//         ROS_ERROR("Command Should be \"set_file_name\" \"start\" or \"stop\" or or \"shutdown\"");
//       }
//     }

//     else("File name needs to be set before sending command to start recording");
// }



// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "frame_recorder_node");
//   thisNodeName = "frame_capture";

//   ros::NodeHandle n;

//   ros::Subscriber controlSub = n.subscribe("recording_control_topic", 10, recordingControlCallback);

//   while(!recordFlag)
//   {
//     ros::spinOnce(); 
//     ROS_ERROR("Frame not recording!");
//   }

//   return 0;

// }
