#include <signal.h>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl_ros/impl/transforms.hpp>

// PCL specific includes
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

#include <pcl/kdtree/kdtree.h>
#include "bimur_robot_vision/TabletopPerception.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detection_testclient");
    if (argc != 1) {
    	ROS_INFO("usage: object_detection_testclient");
    	return 1;
    }

    ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<bimur_robot_vision::TabletopPerception>("object_detection");
	
	bimur_robot_vision::TabletopPerception srv;

	if (client.call(srv)) {
		//ROS_INFO("Plane found: "+ srv.response.is_plane_found);
		ROS_INFO("Objects found: " + (int)srv.response.cloud_clusters.size());
	} else {
		ROS_ERROR("Failed to call service");
		return 1;
	}

	return 0;
}





