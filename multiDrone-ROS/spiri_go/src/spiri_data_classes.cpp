/*
 * This is a collection of classes that are used in spiri_go and that
 * are exposed by boost in spiri_boost so that they can be read and written
 * in stand-alone python scripts as well as ROS.
 */

#include <spiri_data_classes.h>

SpiriAttitude::SpiriAttitude(){
	roll = 0;
	pitch = 0;
	yaw = 0;
}

SpiriAttitude::SpiriAttitude(double roll, double pitch, double yaw){
	this->roll = roll;
	this->pitch = pitch;
	this->yaw = yaw;
}
