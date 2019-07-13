#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;

void cloud_cb1 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Create a container for the data.
  //sensor_msgs::PointCloud2 output;

  // Do data processing here...
  //output = *input;

  // Publish the data.
  //pub.publish (output);


  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output1;
  pcl_conversions::fromPCL(cloud_filtered, output1);

  // Publish the data
  pub1.publish (output1);
}


void cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Create a container for the data.
  //sensor_msgs::PointCloud2 output;

  // Do data processing here...
  //output = *input;

  // Publish the data.
  //pub.publish (output);


  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output2;
  pcl_conversions::fromPCL(cloud_filtered, output2);

  // Publish the data
  pub2.publish (output2);
}


void cloud_cb3 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Create a container for the data.
  //sensor_msgs::PointCloud2 output;

  // Do data processing here...
  //output = *input;

  // Publish the data.
  //pub.publish (output);


  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output3;
  pcl_conversions::fromPCL(cloud_filtered, output3);

  // Publish the data
  pub3.publish (output3);
}


void cloud_cb4 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Create a container for the data.
  //sensor_msgs::PointCloud2 output;

  // Do data processing here...
  //output = *input;

  // Publish the data.
  //pub.publish (output);


  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output4;
  pcl_conversions::fromPCL(cloud_filtered, output4);

  // Publish the data
  pub4.publish (output4);
}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe ("/camera_front/depth/color/points", 1, cloud_cb1);
  ros::Subscriber sub2 = nh.subscribe ("/camera_back/depth/color/points", 1, cloud_cb2);
  ros::Subscriber sub3 = nh.subscribe ("/camera_left/depth/color/points", 1, cloud_cb3);
  ros::Subscriber sub4 = nh.subscribe ("/camera_right/depth/color/points", 1, cloud_cb4);
  // Create a ROS publisher for the output point cloud
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output1", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("output2", 1);
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("output3", 1);
  pub4 = nh.advertise<sensor_msgs::PointCloud2> ("output4", 1);

  // Spin
  ros::spin ();
}
