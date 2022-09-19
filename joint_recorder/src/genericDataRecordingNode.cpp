#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <fstream>
#include <iostream> 
#include <cstdlib>
#include "joint_recorder/recorderMsg.h"
#include "ros_msg_parser/ros_parser.hpp"

std::ofstream file;
bool isFirstRun = true;
std::string thisNodeName;
std::string fileName;
bool recordFlag = false;
ros::Duration dur1s;

void topicCallback(const RosMsgParser::ShapeShifter& msg,
                   const std::string& topic_name,
                   RosMsgParser::ParsersCollection& parsers)
{
  // Register the topic definition.
  // it will not be done twice
  parsers.registerParser(topic_name, msg);
  
  // ROS_ERROR("IN GENERIC CALLBACK FUNCTION:");

  if(recordFlag)
  {  
    auto deserialized_msg = parsers.deserialize(topic_name, msg);
  
    file.open(fileName, std::ios::app);//open file for writing

    if (isFirstRun) //on the first run, saves field names on first row
    {
      for (auto it : deserialized_msg->flat_msg.name) // flat_msg.name => values that are/can be treated as/ strings
      {
        const std::string& key = it.first.toStdString();
        //const std::string& value = it.second;
        
        file << key << ","; //saving only the "key" variable on first row, which is the name of each field
        // ROS_INFO(key.c_str());
      }

      for (auto it : deserialized_msg->renamed_vals) //renamed_vals => values that are NOT strings
      {
        const std::string& key = it.first;
        //double value = it.second;
        file << key << ","; //saving only the "key" variable on first row, which is the name of each field
        // ROS_INFO(key.c_str());
      }
      file << "\n";
      isFirstRun = false;
    }

    //now saving only values of fields on subsequent messages: 

    for (auto it : deserialized_msg->flat_msg.name) 
    {
      //const std::string& key = it.first.toStdString();
      const std::string& value = it.second;
      //std::cout << key << " = " << value << std::endl;
      file << value << ","; 
      // ROS_INFO(value.c_str());
    }

    for (auto it : deserialized_msg->renamed_vals) 
    {
      //const std::string& key = it.first;
      double value = it.second;
      //std::cout << key << " = " << value << std::endl;
      file << value << ",";
      // ROS_INFO(std::to_string(value).c_str());
    }

    file << "\n";
    file.close();
  }
}


//this is the callback function for the recording_control_topic, which is used by the data_recording_service to 
//control when nodes start/stop recording, and the filenames. 
void recordingControlCallback(const joint_recorder::recorderMsg::ConstPtr& msg)
{
    std::string debugString = msg->topic.data + " " + thisNodeName;
    // ROS_ERROR(debugString.c_str());

    if(msg->command.data.compare("set_file_name") == 0 && (msg->topic.data.compare(thisNodeName) == 0))
    {
      fileName = msg->fileName.data;
    }
    else if(!fileName.empty()){ //only starts if filename has been set
      if (msg->command.data.compare("start") == 0)//start recording
      {
          // ROS_INFO("In Start condition of recordService callback function");
          recordFlag = true;
      }
      else if (msg->command.data.compare("stop") == 0)
      {
          // ROS_ERROR("In STOP condition of recordService callback function");
          recordFlag = false;
      }
      else if (msg->command.data.compare("shutdown") == 0)
      {
          // ROS_ERROR("Shutting down");
          ros::shutdown();
      }
      else{
          // ROS_ERROR("Command Should be \"set_file_name\" \"start\" or \"stop\" or or \"shutdown\"");
      }
    }
    else("File name needs to be set before sending command to start recording");

}



// usage: pass the name of the topic as command line argument
int main(int argc, char** argv)
{
  RosMsgParser::ParsersCollection parsers;

  const std::string topic_name = argv[1];
  thisNodeName = topic_name;
  fileName = "";

  ros::init(argc, argv, "generic_data_recording_node");
  ros::NodeHandle nh;

  ros::Subscriber controlSub = nh.subscribe("recording_control_topic", 10, recordingControlCallback);
  

  //lambda and boost::function from facontidavide (ros_msg_parser) creator
  boost::function<void(const RosMsgParser::ShapeShifter::ConstPtr&)> callback;
  
  callback = [&parsers, topic_name](const RosMsgParser::ShapeShifter::ConstPtr& msg) -> void { topicCallback(*msg, topic_name, parsers); };
  ros::Subscriber subscriber = nh.subscribe(topic_name, 1000, callback);
  
  std::string statusListening = "Listening to " + thisNodeName;
  
  while(ros::ok())
  {
    // ROS_ERROR(statusListening.c_str()); 
    ros::spinOnce();
  } 
  
  return 0;
}
