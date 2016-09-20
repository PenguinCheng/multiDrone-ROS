
/*******************************************************************************
 *  The Spiri Project
 *
 *  File: leader.cpp
 *
 *  Purpose: Formation control of leaders with collision avoidance using releative position.
 *
 *  @author Peng Cheng
 *  @version 0.1.0    2016/4/06
 ******************************************************************************/

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <spiri_go/TakeoffAction.h>
#include "spiri_go.h"
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <sstream>
#include <px4ros/gpsPosition.h>
#include <math.h>
#include <fstream>


//global variables
sensor_msgs::NavSatFix selfGlobalPos;    //self gps subscribed from mavros
px4ros::gpsPosition neigGlobalPos;       //neighbour gps subscribed from leader_control_decode.py

#define R  6371000         //meter
#define PI 3.1415926535898

/*******************************************************************************
 *  Callback function on the gps of the copter
 *
 *  @param gpsPositionConstPtr& the gps callback pointer
 ******************************************************************************/
void gpsSubCb(const sensor_msgs::NavSatFixConstPtr& msg){
   selfGlobalPos = *msg;
}

/*******************************************************************************
 *  Callback function on the gps of the neighbour copter
 *
 *  @param gpsPositionConstPtr& the gps callback pointer
 ******************************************************************************/
void neigGpsSubCb(const px4ros::gpsPositionConstPtr& msg)
{
    neigGlobalPos = *msg;
}

/*******************************************************************************
 *  Main function
 *
 *  @param 
 ******************************************************************************/
int main(int argc, char **argv) {
	ros::init(argc, argv, "spiri_go_client");
	ros::NodeHandle nh;

	ros::Rate loop_rate(10.0);

	// Subscriber to the Copter gps
	string gps_ = nh.resolveName("/mavros/global_position/raw/fix");
	ros::Subscriber gps = nh.subscribe(gps_, 5, gpsSubCb);

	// Subscriber to the neighbour gps
	string neigGps_ = nh.resolveName("neighbour_position");
	ros::Subscriber neigGps = nh.subscribe(neigGps_, 5, neigGpsSubCb);	

	//Publisher to the gps(toString)
	string gps_string_pub_ = nh.resolveName("serial_data_send");
	ros::Publisher gps_string_pub = nh.advertise<std_msgs::String>(gps_string_pub_, 5);

	while(nh.ok()) {
		//1.publish self gps(toString)
		std_msgs::String gps_string;
		std::stringstream ss;
		//ss << "01o"<<selfGlobalPos.longitude<<"a"<<selfGlobalPos.latitude<<"#";
		ss << "x1o"<<120.87654321<<"a"<<30.12345678<<"#";
		gps_string.data = ss.str();
		gps_string_pub.publish(gps_string);

		ROS_INFO("self_lat:%f", selfGlobalPos.latitude);
		ROS_INFO("self_lon:%f", selfGlobalPos.longitude);

		//2.read neighbour gps
		ROS_INFO("Neigb_lat:%f", neigGlobalPos.lat);
		ROS_INFO("Neigb_lon:%f", neigGlobalPos.lon);
		
		ros::spinOnce();
    		loop_rate.sleep();
	}

	return 0;

}
