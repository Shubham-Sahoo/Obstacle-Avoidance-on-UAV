#include "ros/ros.h"
#include "mavros_msgs/SetMavFrame.h"
#include "mavros_msgs/CommandBool.h"
#include "geometry_msgs/Twist.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/SetMode.h"
#include <cstdlib>

geometry_msgs::Twist v;
mavros_msgs::PositionTarget vel_msg;
int count=0;
int main(int argc,char **argv)
{
	ros::init(argc,argv,"Frame_set_client");
	if(argc!=4)
	{
		return 1;

	}
	ros::NodeHandle n;
	//ros::ServiceClient client=n.serviceClient<mavros_msgs::SetMavFrame>("/mavros/setpoint_position/mav_frame");
	ros::Publisher pub= n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",100);

	ros::Rate loop_rate(10);
	vel_msg.coordinate_frame=mavros_msgs::PositionTarget::FRAME_BODY_NED;
	vel_msg.header.frame_id="drone";
        vel_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                            mavros_msgs::PositionTarget::IGNORE_PY |
                            mavros_msgs::PositionTarget::IGNORE_PZ |
                            mavros_msgs::PositionTarget::IGNORE_AFX |
                            mavros_msgs::PositionTarget::IGNORE_AFY |
                            mavros_msgs::PositionTarget::IGNORE_AFZ |
                            mavros_msgs::PositionTarget::FORCE |
                            mavros_msgs::PositionTarget::IGNORE_YAW |
                            mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;
	vel_msg.header.stamp=ros::Time::now();
	ROS_INFO("zero velocity");
    	for (int i = 50; ros::ok() && i > 0; --i)
    	{	
		vel_msg.velocity.x=0;//v.linear.x;
		vel_msg.velocity.y=0;//v.linear.y;
		vel_msg.velocity.z=0;//v.linear.z;
      		pub.publish(vel_msg);
      		ros::spinOnce();
      		// rate.sleep();
      		ros::Duration(0.01).sleep();
    		}
    	ROS_INFO("Done with zero velocity set");
	ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	mavros_msgs::SetMode offb_set_mode;
  	offb_set_mode.request.custom_mode = "OFFBOARD";
  	if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
  		ROS_INFO("OFFBOARD enabled");
  	else
  	{
    		ROS_INFO("unable to switch to offboard");
    		return -1;
  	}
	while(ros::ok())
	{	
		
		vel_msg.header.stamp=ros::Time::now();
		vel_msg.velocity.x=0;//v.linear.x;
		vel_msg.velocity.y=0;//v.linear.y;
		vel_msg.velocity.z=count;//v.linear.z;
		ROS_INFO("Aaa gya !!!");
		for (int i = 10; ros::ok() && i > 0; --i)
    		{
      			pub.publish(vel_msg);
      			ros::spinOnce();
      			// rate.sleep();
      			ros::Duration(0.01).sleep();
    		}
    		
		if(count>=5)
		{	
			count=-10;
		}
		else if(count<=5)	
		{	
			count+=1;
		}
		
		ROS_INFO("%d",&count);
	}
	/*mavros_msgs::SetMavFrame srv;
	srv.request.mav_frame = atoll(argv[1]);
	if(client.call(srv))
	{
		ROS_INFO("Frame set Successful.  %ld",(long int)srv.response.success);
		
	}
	else
  	{
    		ROS_ERROR("Failed to call service Frame set");
    		return 1;
  	}
	
	*/
	return 0;
}
