#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"


using namespace std;
using namespace cv;

int main(int argc, char **argv){
    geometry_msgs::PoseStamped my_vidoe2;
    ros::init(argc, argv, "coordinator");

    ros::NodeHandle n;
    ros::Publisher positions_pub = n.advertise<geometry_msgs::PoseStamped>("/gps", 1000);


	
ros::Rate loop_rate(30);
        int count=0;
    while (ros::ok())
    {
            double x;



    	geometry_msgs::PoseStamped gps;
  gps.header.stamp=msg->header.stamp;

  gps.status.status=msg->status.status;

  gps.latitude=msg->latitude;

  gps.longitude=msg->longitude;

  gps.altitude=msg->altitude;

  gps.position_covariance[0]=msg->position_covariance[0];
  gps.position_covariance[4]=msg->position_covariance[4];
  gps.position_covariance[8]=msg->position_covariance[8];

	ros::spinOnce();

        loop_rate.sleep();
        ++count;
        }
