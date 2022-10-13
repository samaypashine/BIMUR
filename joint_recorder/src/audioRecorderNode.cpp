#define ALSA_PCM_NEW_HW_PARAMS_API

#include <alsa/asoundlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <fstream>
#include <iostream> 
#include <cstdlib>
#include "joint_recorder/recorderMsg.h"

bool recordFlag = false;
long loops;
int rc;
int size;
snd_pcm_t *handle;
snd_pcm_hw_params_t *params;
unsigned int val;
int dir;
snd_pcm_uframes_t frames;
char *buffer;
std::ofstream file; 
std::string fileName, thisNodeName;

void recordingControlCallback(const joint_recorder::recorderMsg::ConstPtr& msg) {
    if ((msg->command.data.compare("set_file_name") == 0) && (msg->topic.data.compare(thisNodeName) == 0)) {
      fileName = msg->fileName.data;
    }
    else if (!fileName.empty()) { // only starts if filename has been set
      if (msg->command.data.compare("start") == 0) { //start recording
        ROS_INFO("In Start condition of recordService callback function");
        recordFlag = true;

        /* Open PCM device for recording (capture). */
        rc = snd_pcm_open(&handle, "default", SND_PCM_STREAM_CAPTURE, 0);
        if (rc < 0) {
          fprintf(stderr, "unable to open pcm device: %s\n", snd_strerror(rc));
          exit(1);
        }

        /* Allocate a hardware parameters object. */
        snd_pcm_hw_params_alloca(&params);

        /* Fill it in with default values. */
        snd_pcm_hw_params_any(handle, params);

        /* Set the desired hardware parameters. */

        /* Interleaved mode */
        snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);

        /* Signed 16-bit little-endian format */
        snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);

        /* Two channels (stereo) */
        snd_pcm_hw_params_set_channels(handle, params, 2);

        /* 44100 bits/second sampling rate (CD quality) */
        val = 44100;
        snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir);

        /* Set period size to 32 frames. */
        frames = 32;
        snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

        /* Write the parameters to the driver */
        rc = snd_pcm_hw_params(handle, params);
        if (rc < 0) {
          fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(rc));
          exit(1);
        }

        /* Use a buffer large enough to hold one period */
        snd_pcm_hw_params_get_period_size(params, &frames, &dir);
        size = frames * 4; /* 2 bytes/sample, 2 channels */
        buffer = (char *) malloc(size);

        /* We want to loop for 5 seconds */
        snd_pcm_hw_params_get_period_time(params, &val, &dir);
        loops = 10000000 / val;

        file.open(fileName, std::ios::app);

        while (recordFlag && ros::ok()) { //This loop invokes snd_pcm_readi() to capture audio frames from mic and sends each frame to buffer
          // loops--;
          // ROS_ERROR("Audio recording in progress.");
          rc = snd_pcm_readi(handle, buffer, frames);
          if (rc == -EPIPE) {
            /* EPIPE means overrun */
            fprintf(stderr, "overrun occurred\n");
            snd_pcm_prepare(handle);
          } else if (rc < 0) {
            fprintf(stderr, "error from read: %s\n", snd_strerror(rc));
          } else if (rc != (int)frames) {
            fprintf(stderr, "short read, read %d frames\n", rc);
          }
          rc = write(1, buffer, size);
          if (rc != size)
            fprintf(stderr, "short write: wrote %d bytes\n", rc);
          
          for (int i = 0; i < size; i++) { //for-loop to save buffer to file
            file << *(buffer + i); 
          }
          ros::spinOnce();
        }
        
        // ROS_ERROR("End of start condition. IT SHOULD NOT BE RECORDING HERE.");
        file.close();

      }
      else if (msg->command.data.compare("stop") == 0) {
          // ROS_ERROR("In STOP condition of recordService callback function"); // Stop recording but keep node running
          snd_pcm_drain(handle);
          snd_pcm_close(handle);
          free(buffer);
          recordFlag = false;
      }
      else if (msg->command.data.compare("shutdown") == 0) { // shut node down
          // ROS_ERROR("Shutting down");
          snd_pcm_drain(handle);
          snd_pcm_close(handle);
          free(buffer); 
          ros::shutdown();
      }
      else {
          // ROS_ERROR("Command Should be \"set_file_name\" \"start\" or \"stop\" or or \"shutdown\"");
      }
    }
    else("File name needs to be set before sending command to start recording");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "audio_recorder_node");
  thisNodeName = "audio_capture";

  ros::NodeHandle n;
  ros::Subscriber controlSub = n.subscribe("recording_control_topic", 10, recordingControlCallback);

  while (!recordFlag) {
    ros::spinOnce(); 
    // ROS_ERROR("Audio not recording!");
  }

  snd_pcm_drain(handle);
  snd_pcm_close(handle);
  free(buffer);

  return 0;
}
