#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "std_msgs/String.h"
#include "stdio.h"

using namespace std;

int land = 0; // Emergency Landing
int rtl = 0;  // Emergency RTL
int mode = 0;  // Mode
int disarm = 0;  // Emergency Disarm
int arm = 0;   // ARMING

double joy_vz = 0;
double joy_yaw = 0;
double joy_vx = 0;
double joy_vy = 0;
int prev_mode1 = 0; // DEFAULT-0 : 0:OFFBOARD | 1: OFFBOARD-RC_Overide | -1: STABALIZED
int off_RcOveride = 0; // In OFFBOARD mode Priority control is RC over Planned Mission

#define axes_disarm 6
#define axes_thrust 0
#define axes_yaw 3
#define axes_roll 1
#define axes_pitch 2
#define axes_emg1 7 // Emergency Mode Switch : Position Hold | Alt Hold
#define axes_emg2 4 // Emergency Mode Switch : RTL | Land
#define axes_mode1_pin 5 // 0) MANUAL | 1)OFFBOARD |
#define button_mode2 1 

#define scale_axes_disarm 1
#define scale_axes_thrust 1 //Important Scaling Factor
#define scale_axes_yaw 1
#define scale_axes_roll 1
#define scale_axes_pitch 1
#define scale_axes_emg1 1 
#define scale_axes_emg2 1 
#define scale_axes_mode1 1
#define scale_axes_mode2 1

bool fcu_stat = 0;
bool armed_status = 0;
bool guided_mode = 0;	
int rtl_act=0;
int land_act=0;
int axes_mode1=1;
int no_com=1;


void drone_state(const mavros_msgs::State::ConstPtr& msg)
{
	fcu_stat = (bool)msg->connected;
	armed_status = (bool)msg->armed;
	guided_mode = (bool)msg->guided;
	
}




mavros_msgs::SetMode mode_srv;
mavros_msgs::PositionTarget vel_msg;

int joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	joy_vz = (double)joy->axes[axes_thrust];
	ROS_INFO("%f",&joy_vz);
	joy_yaw = (float)joy->axes[axes_yaw];
	joy_vy = (double)joy->axes[axes_pitch];
	joy_vx = (double)joy->axes[axes_roll];
	disarm = (int)joy->axes[axes_disarm];		//axes[axes_disarm] : 1: EMERGENCY DISARM | -1: ARMING | 0: Do Nothing
	if(joy->axes[axes_emg2]== 1)               //axes[axes_emg2] :  -1:RTL | 1:Land | 0: Do Nothing
	{											//axes[axes_emg1] :  NOT USED | NOT USED | 0: Do Noting
		rtl_act=1;

	}
	else if(joy->axes[axes_emg2]== -	1)
	{
		land_act=1;
	}
	else
	{
		no_com=1;

	}
	axes_mode1 = (int)joy->axes[axes_mode1_pin];  //axes[axes_mode1_pin] : 0: OFFBOARD | 1: OFFBOARD-RC_Overide | -1: STABALIZED
	ROS_INFO("%d",&axes_mode1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Joy_control");
  	ros::NodeHandle n;
	ros::ServiceClient client1 = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	ros::ServiceClient client2 = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");	
	ros::Subscriber sub = n.subscribe("/mavros/state", 50, drone_state);											
	ros::Publisher pub2 = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",50);
	ros::Publisher pub1 = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",50);
	ros::Publisher pub= n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",50);
	ros::Subscriber joy_sub_manual = n.subscribe<sensor_msgs::Joy>("/joy", 100, &joyCallback);
    mavros_msgs::CommandBool srv1;

    ros::Rate loop_rate(100);

 //    for (int i = 30; ros::ok() && i > 0; --i)
	// {	
	// 	vel_msg.velocity.x=0;//v.linear.x;
	// 	vel_msg.velocity.y=0;//v.linear.y;
	// 	vel_msg.velocity.z=0;//v.linear.z;
	// 	pub2.publish(vel_msg);
	// 	ros::spinOnce();
	// 	// rate.sleep();
			      			
	// }
 //    mode_srv.request.base_mode=216;
	// mode_srv.request.custom_mode="OFFBOARD";
	// if(client2.call(mode_srv) && mode_srv.response.mode_sent)
	// {
	// 	ROS_INFO("OFFBOARD ENABLED...");
	// }

  	while(ros::ok())
  	{
  		/*######## EMERGENNCY : DISARM [TOP PRIORITY CHECK]  ###########*/
  		// For Disarming : To be used at Last in Emergency Situation
  		// This will make the Drone FALL DEAD
  		if(disarm == 1 && armed_status == 1)
		{  //DISARM if Disarm Button ON
			//in Disarm funct check status: If not Disarm then Disarm
			
			srv1.request.value = 0;

			if(client1.call(srv1))
			{
				ROS_INFO("Disarm Successful.  %ld",(long int)srv1.response.success);
				armed_status = 0;
			}
			else
  			{
    			ROS_ERROR("Failed to call service Disarm !!!");
    			int count=0;
    			int success=0;
    			while(success || (count < 20))
    			{	if(client1.call(srv1))	
    				{	
    					success=1;
    					ROS_INFO("Disarm Successful.  %ld",(long int)srv1.response.success);
						armed_status = 0;
					}
					else
					{
						ROS_INFO("DISARM FAILED.  %d/20 Times",(int)count);
						//ROS_INFO(" TAKE APPROPIATE ACTION");
						armed_status = 1;
					}
				}
    			return 1;
  			}
		}			
		// FOR ARMING the DRONE	
		else if(disarm == -1 && armed_status == 0)
		{
			//check status : if status = DISARMED and Switch is 0 : ARM the Drone
			
			srv1.request.value = 1;
			if(client1.call(srv1))
			{
				ROS_INFO("Armed Successfully.  %ld",(long int)srv1.response.success);
				armed_status = 1;
			}
			else
  			{
    			ROS_ERROR("Failed to call service Arm !!!");
    			return 1;
  			}
		}
	 	// joy->axes[axes_disarm] == 0 : Do Nothing (Neither arm nor disarm)


		// ENABLE EMERGENCY MODES : 
		// ORDER OF EMERGENCY :
		// Emergency 1 : Position Hold -> Alt Hold
		// Emergency 2 : RTL -> Land
		// Emergency 3 : DISARM

		//[IMPORTANT] : check emergency situation 1 : Position Hold -> Altitude Hold
		// If Position Hold Fails then goto Altitude Hold
		// If both Fails : Ask User to Enable emergency situation 2 : RTL -> Land
		// If RTL fails goto Land
		// if both Fails : Ask User to DISARM
		// if Failed use ESTOP if ANY (using Microcontroler Switch for On-board Power)

		// Position Hold Alt Hold RTL Land 
		if(rtl_act == 1 && armed_status == 1)        //RTL
		{	
			mavros_msgs::SetMode srv3;
			srv3.request.base_mode=4;
			srv3.request.custom_mode="RTL";
			client2.call(srv3);
				//Check if AUTO RTL Enabled
				//if Not try for 10 times
				//if failed try goto: Land : land_act = 1 : return

			//while(armed_status==1);
			//srv3.request.custom_mode="OFFBOARD";
		}	
		else if(land_act == 1 && armed_status == 1)			//LAND
		{
			mavros_msgs::SetMode srv4;
			srv4.request.base_mode=4;
			srv4.request.custom_mode="LAND";
			client2.call(srv4);
				//Check if AUTO LAND Acti
			//while(armed_status);			
		}
		else if(no_com==1)
		{

			mavros_msgs::SetMode srv5;
			srv5.request.base_mode=4;
			srv5.request.custom_mode="GUIDED";
			client2.call(srv5);

		}
		/* ### END : EMERGENCY CHECK ### */

		/* MODE OF Operation */
		if(axes_mode1!= prev_mode1)
		{ //Mode Switch 
			// Set Default value of Prev_Mode1 as Global
		
			switch(axes_mode1)
			{
				case -1:
						off_RcOveride = 0;
						mode_srv.request.base_mode=4;
						mode_srv.request.custom_mode="STABILIZE";
						client2.call(mode_srv);
						//switch to manual mode Stabalize
						break;
			
				case 0:	
						off_RcOveride = 0;
				
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
			    		for (int i = 200; ros::ok() && i > 0; --i)
			    		{	
							vel_msg.velocity.x=0;//v.linear.x;
							vel_msg.velocity.y=0;//v.linear.y;
							vel_msg.velocity.z=0;//v.linear.z;
			      			pub2.publish(vel_msg);
			      			ros::spinOnce();
			      			// rate.sleep();
			      			
			    		}
			    		ROS_INFO("Done with zero velocity set");
						mode_srv.request.base_mode=4;
						mode_srv.request.custom_mode="GUIDED";
						if(client2.call(mode_srv) && mode_srv.response.mode_sent)
						{
							ROS_INFO("OFFBOARD ENABLED...");
						}
						else
			  			{
			    			ROS_INFO("unable to switch to offboard");
			    			return -1;
			  			}
						//switch to guided mode
						break;
						
				case 1:	// Offboard RC Overide
						off_RcOveride = 1;
					
									//get current pose
						if(disarm == -1 && armed_status == 0)
						{
							//check status : if drone_status = DISARMED and Switch is -1(ARMED) : ARM the Drone
			
							srv1.request.value = 1;
							if(client1.call(srv1))
							{
								ROS_INFO("Armed Successfully.  %ld",(long int)srv1.response.success);
								armed_status = 1;
							}
							else
  							{
    							ROS_ERROR("Failed to call service Arm !!!");
    							return 1;
  							}
  						}
							
						{
							vel_msg.coordinate_frame=mavros_msgs::PositionTarget::FRAME_BODY_NED;
							vel_msg.header.frame_id="drone";
			        		vel_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
			                   			        mavros_msgs::PositionTarget::IGNORE_PY |
			                        			mavros_msgs::PositionTarget::IGNORE_PZ |
			                    			    mavros_msgs::PositionTarget::IGNORE_AFX |
			                        		    mavros_msgs::PositionTarget::IGNORE_AFY |
			                        		    mavros_msgs::PositionTarget::IGNORE_AFZ |
			                        		    mavros_msgs::PositionTarget::FORCE |
			                        		    mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;
							vel_msg.header.stamp=ros::Time::now();
							ROS_INFO("zero velocity");
			    			for (int i = 50; ros::ok() && i > 0; --i)
			    			{	
								vel_msg.velocity.x=0;//v.linear.x;
								vel_msg.velocity.y=0;//v.linear.y;
								vel_msg.velocity.z=0;//v.linear.z;
								vel_msg.yaw=0;
			      				pub.publish(vel_msg);
			      				ros::spinOnce();
			      				// rate.sleep();
			      				
			    			}
			    			ROS_INFO("Done with zero velocity set");
							
			  				mode_srv.request.custom_mode = "GUIDED";
			  				if (client2.call(mode_srv) && mode_srv.response.mode_sent)
			  				{
			  					ROS_INFO("OFFBOARD enabled");
			  				}
			  				else
			  				{
			    				ROS_INFO("unable to switch to offboard");
			    				return -1;
			  				}
												
						
						}	
						break;
						
				default:off_RcOveride = 0;
						ROS_INFO("INVALID MODE INPUT");
			}
		}	
		
		else if(axes_mode1==1)
		{
				vel_msg.header.stamp=ros::Time::now();
				vel_msg.velocity.x=(double)joy_vx;//v.linear.x;
				vel_msg.velocity.y=(double)joy_vy;//v.linear.y;
				vel_msg.velocity.z=(double)joy_vz;//v.linear.z;
				vel_msg.yaw=(float)joy_yaw;
				//ROS_INFO("Aaa gya !!!");
				for (int i = 50; ros::ok() && i > 0; --i)
				{
					pub.publish(vel_msg);
				}
		}	
		prev_mode1 = axes_mode1;
	
    ros::spinOnce();
    // rate.sleep();
    ros::Duration(0.001).sleep();
	}  
    return 0;  
}
