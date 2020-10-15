#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>

#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <experimental/filesystem> 
#include <chrono>
#include <thread>

//namespace stdfs = std::experimental::filesystem ;
using namespace std;
using namespace Eigen;
using namespace pcl;
ros::Publisher rospoint_pub;
using recursive_directory_iterator = std::experimental::filesystem::recursive_directory_iterator;
int main(int argc, char** argv)
{
	ros::init (argc, argv, "pcd2bag");
 	ros::NodeHandle nh;
  	std::string filename_in;
 	std::string topicname_in;
	 int time;
  	ros::NodeHandle nn("~");
	
	nn.getParam("filename", filename_in);
	nn.getParam("topicname", topicname_in);
	nn.getParam("hertz", time);
	vector<std::experimental::filesystem::v1::path> pathvector;
    rospoint_pub = nh.advertise<sensor_msgs::PointCloud2> (topicname_in, 1);
		//pcd file reader    
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& dirEntry : recursive_directory_iterator(filename_in))
	{
	   pathvector.push_back(dirEntry.path());
	   std::cout << dirEntry << std::endl;
	}
	sort(pathvector.begin(), pathvector.end()); 
	for(int i =0; i<= pathvector.size(); i++)
	{
	std::cout << pathvector[i] << std::endl;
	pcl::io::loadPCDFile<pcl::PointXYZ> (pathvector[i], *cloud);
	sensor_msgs::PointCloud2 output;
 	pcl::toROSMsg(*cloud, output);
	output.header.frame_id= "base_link";
	output.header.stamp =ros::Time::now();
 	rospoint_pub.publish(output);
	 sleep(1);
	}	

    return 0;
}
