
/*******************************************************************************
 *  The Spiri Project
 *
 *  File: takeoff_and_land.cpp
 *
 *  Purpose: simpely takeoff and land for autonomous flights.
 *
 *  @author Peng Cheng
 *  @version 0.1.0 2015/12/24
 *  @version 0.1.1 2016/03/16
 ******************************************************************************/

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <spiri_go/TakeoffAction.h>
#include <spiri_go/LandHereAction.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "spiri_go_client");

	//It's better to wait sonme time, for this file has no NodeHandle, we should add ros::Time::init()
	ros::Time::init();
	ros::Duration(3).sleep();
	
	actionlib::SimpleActionClient<spiri_go::TakeoffAction> ac("spiri_take_off", true);      //first param: server name, second param: true means a new thread
	actionlib::SimpleActionClient<spiri_go::LandHereAction> ac1("spiri_land_here", true);  //first param: server name, second param: false means not to use threads

	ROS_INFO("Waiting for action server(takeOff) to start.");

	ac.waitForServer();	
	
	ROS_INFO("Action server started--Taking off!");

	spiri_go::TakeoffGoal goal;

	//set the target height to take off!
	goal.height = 3;
	ac.sendGoal(goal);

	//wait for action(takeOff) to return
	bool is_takeOff_finished = ac.waitForResult(ros::Duration(60.0));

	if(is_takeOff_finished ) {
		ROS_INFO("reached target height!");

		//do something
		ros::Duration(20).sleep();
	}
	else {
		ROS_INFO("should set mode Land!");
	}

	//land
	ROS_INFO("Waiting for action server(land) to start.");
	ac1.waitForServer();	
	ROS_INFO("Start to land!.");
	
	spiri_go::LandHereGoal goal_1;
	goal_1.height = 0;
	ac1.sendGoal(goal_1);

	//wait for action(land) to return
	bool is_land_finished = ac1.waitForResult(ros::Duration(60.0));
	if(is_land_finished ) {
		ROS_INFO("Land success!");
	}

	return 0;

}
