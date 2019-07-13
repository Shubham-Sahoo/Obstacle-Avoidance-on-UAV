#include "ros/ros.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/CommandBool.h"
#include <cstdlib>

int main(int argc,char **argv)
{
	ros::init(argc,argv,"Land_client");
	if(argc!=6)
	{
		return 1;

	}
	ros::NodeHandle n;
	ros::ServiceClient client=n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
	ros::ServiceClient client2=n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	mavros_msgs::CommandTOL srv;
	srv.request.min_pitch = atoll(argv[1]);
	srv.request.yaw = atoll(argv[2]);
	srv.request.latitude = atoll(argv[3]);
	srv.request.longitude = atoll(argv[4]);
	srv.request.altitude = atoll(argv[5]);
	if(client.call(srv))
	{
		ROS_INFO("Landed Successfully.  %ld",(long int)srv.response.success);
		
				
		/* Disarm Service------>
		mavros_msgs::CommandBool over;
		over.request.value=0;
		if(client2.call(over))
		{
			ROS_INFO("Disarmed Successfully.....!!!");
		}
		else
		{
			ROS_INFO("Error Disarming bot...");
		}*/
	}
	else
  	{
    		ROS_ERROR("Failed to call service Land");
    		return 1;
  	}
	

	return 0;
}
