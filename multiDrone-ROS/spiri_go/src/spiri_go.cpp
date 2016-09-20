// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*******************************************************************************
 *  The Spiri Project
 *
 *  File: spiri_go.cpp
 *
 *  Purpose: Sets up the ROS node that serves as the service 
 *  and action server for autonomous flights.
 *
 *  @author Skyler Olson
 *  @author John Lian
 *  @author Peng Cheng
 *  @version 0.1.0 27/11/15
 *  @version 0.1.1 2016/03/16
 ******************************************************************************/

#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/StreamRate.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandBool.h"
#include "std_msgs/String.h"
#include "spiri_go.h"
#include <sys/stat.h>
#include <string>

/*******************************************************************************
 *  The SpiriGo constructor. The constructor will set up subscribers and 
 *  publishers to mavros.
 ******************************************************************************/
SpiriGo::SpiriGo():
takeoff_as(nh, "spiri_take_off", boost::bind(&SpiriGo::armAndTakeOff, this, _1), false),
land_here_as(nh, "spiri_land_here", boost::bind(&SpiriGo::landHere, this, _1), false)
{
    ROS_INFO("Constructing Go");

    // Subscriber to the position of the copter centered on the home location
    string local_ = nh.resolveName("/mavros/local_position/pose");
    local = nh.subscribe(local_, 1, &SpiriGo::localSubCb, this);

    // Subscriber to the states of the copter
    string state_ = nh.resolveName("/mavros/state");
    state = nh.subscribe(state_, 1, &SpiriGo::stateSubCb, this);

    // Subscriber to gps of copter, I add
    string gps_ = nh.resolveName("/mavros/global_position/raw/fix");
    gps = nh.subscribe(gps_, 1, &SpiriGo::gpsSubCb, this);

    // Subscriber to height of copter, I add
    string alt_ = nh.resolveName("/mavros/global_position/global");
    alt = nh.subscribe(alt_, 1, &SpiriGo::altSubCb, this);

    //Subscriber to the mavlink/from of the copter, I add
    string connect_state_ = nh.resolveName("/mavlink/from");
    connect_state = nh.subscribe(connect_state_, 1, &SpiriGo::connectSubCb, this);

    // Control the velocity of the copter
    string vel_ = nh.resolveName("/mavros/setpoint_velocity/cmd_vel");
    vel = nh.advertise<geometry_msgs::TwistStamped>(vel_,1);

    // Set the mode 
    string set_mode_ = nh.resolveName("/mavros/set_mode");
    set_mode = nh.serviceClient<mavros_msgs::SetMode>(set_mode_);

    // Set the stream rate
    string set_stream_rate_ = nh.resolveName("/mavros/set_stream_rate");
    set_stream_rate = nh.serviceClient<mavros_msgs::StreamRate>(set_stream_rate_);

    // Arming
    string arm_ = nh.resolveName("/mavros/cmd/arming");
    arm = nh.serviceClient<mavros_msgs::CommandBool>(arm_);

    // Tell the copter to take off autonomously
    string takeoff_ = nh.resolveName("/mavros/cmd/takeoff");
    takeoff = nh.serviceClient<mavros_msgs::CommandTOL>(takeoff_);

    // MAVLink commander for yaw because setAttitude doesn't work in APM
    string mavlink_cmd_srv_ = nh.resolveName("/mavros/cmd/command");
    mavlink_cmd_srv = nh.serviceClient<mavros_msgs::CommandLong>(mavlink_cmd_srv_);

    // Create the services and actions that other nodes can interact with spiri_go through
    getLocalPositionService = nh.advertiseService("spiri_local_position", &SpiriGo::getLocalPositionSCB, this);
    getLastStateService = nh.advertiseService("spiri_state", &SpiriGo::getLastStateSCB, this);

    // State variables
    last_state.armed = false;
    last_state.connected = false;
    last_state.guided = false;
    last_state.mode = "STABILIZE";
    taking_off = false;
    flying = false;

    // location z has to start here so you don't set flying too soon
    location.position.z = 0;

    // set the velocity to 0 all around
    velocity.linear.x = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;

    ROS_INFO("Constructed Go");

}

/*******************************************************************************
 *  The SpiriGo destructor
 ******************************************************************************/
SpiriGo::~SpiriGo()
{

}

////////////////////////////////////////////////////////////////////////////////
// Callback functions
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *  Callback function on the local position of the copter
 *
 *  @param loalPtr the location callback pointer
 ******************************************************************************/
void SpiriGo::localSubCb(const geometry_msgs::PoseStamped localPtr)
{
    location = localPtr.pose;
    last_location = localPtr.pose;
}

/*******************************************************************************
 *  Callback function on the state of the copter
 *
 *  @param statePtr the state callback pointer
 ******************************************************************************/
void SpiriGo::stateSubCb(const mavros_msgs::State statePtr)
{
    last_state = statePtr;
}

/*******************************************************************************
 *  Callback function on the gps of the copter
 *
 *  @param NavSatFixConstPtr the state callback pointer
 ******************************************************************************/
void SpiriGo::gpsSubCb(const sensor_msgs::NavSatFixConstPtr& msg)
{
    last_gps = *msg;
}

/*******************************************************************************
 *  Callback function on the height of the copter
 *
 *  @param NavSatFixConstPtr the state callback pointer
 ******************************************************************************/
void SpiriGo::altSubCb(const sensor_msgs::NavSatFixConstPtr& msg)
{
    last_alt = *msg;
    ROS_INFO("altitude is %f", last_alt.altitude);
}

/*******************************************************************************
 *  Callback function on the /mavlink/from of the copter
 *
 *  @param NavSatFixConstPtr the state callback pointer
 ******************************************************************************/
void SpiriGo::connectSubCb(const mavros_msgs::MavlinkPtr& fromPtr)
{
    last_connect_state = *fromPtr;
//ROS_INFO("is_valid:%d",last_connect_state.is_valid);
//ROS_INFO("seq:%d",last_connect_state.seq);
}

/*******************************************************************************
 *  Service callback function on the local position of the copter
 *
 *  @return True if the service is successfully called 
 ******************************************************************************/
bool SpiriGo::getLocalPositionSCB(spiri_go::LocalPosition::Request &req, 
                                  spiri_go::LocalPosition::Response &rsp){
    rsp.x = location.position.x;
    rsp.y = location.position.y;
    rsp.z = location.position.z;
    return true;
}

/*******************************************************************************
 *  Service callback function on the state of the copter
 *
 *  @return True if the service is successfully called 
 ******************************************************************************/
bool SpiriGo::getLastStateSCB(spiri_go::LastState::Request &req, 
                              spiri_go::LastState::Response &rsp){
    rsp.armed = last_state.armed;
    rsp.connected = last_state.connected;
    rsp.guided = last_state.guided;
    rsp.mode = last_state.guided;
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Spiri control methods 
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *  A function to set the desired mode of the copter
 *
 *  @param targetMode is a string for the name of the desired mode
 *      Note: the function will fail if the targetMode name is not valid
 *
 *  @return True if the mode set is successfully called 
 ******************************************************************************/
bool SpiriGo::setMode(const char* targetMode)
{
    mavros_msgs::SetMode modeCmd;

    modeCmd.request.base_mode = 0;
    modeCmd.request.custom_mode = targetMode;

    if(set_mode.call(modeCmd)){
        ROS_INFO("Set to %s Mode.", targetMode);
        return modeCmd.response.success;
    }else{
        ROS_INFO("Failed to set to %s Mode. Currently in %s mode", targetMode, last_state.mode.c_str());
        return false;
    }
}

/*******************************************************************************
 *  A function to set the desired mode of the copter
 *
 *  @param streamID is a string for the name of the desired stream
 *         rate is the desired stream rate
 *      Note: the function will fail if the targetMode name is not valid
 *
 *  @return True if the mode set is successfully called 
 ******************************************************************************/
bool SpiriGo::setStreamRate(unsigned char streamID, int rate)
{
    mavros_msgs::StreamRate streamRateCmd;

    streamRateCmd.request.stream_id = streamID;
    streamRateCmd.request.message_rate = rate;
    streamRateCmd.request.on_off = (rate!=0);

    if(set_stream_rate.call(streamRateCmd)){
        ROS_INFO("Set stream rate success!");
        return true;
    }
    else
    {
        ROS_INFO("Set stream rate failed!");
        return false;
    }
}

/*******************************************************************************
 *  A function to the copter to GUIDED mode
 *
 *  @return True if the the copter has been successfully set to GUIDED mode
 ******************************************************************************/
bool SpiriGo::setGuided()
{
    return setMode("GUIDED");
}

/*******************************************************************************
 *  A function to arm the motors of the copter
 *
 *  @return True if the the copter has been successfully armed
 ******************************************************************************/
bool SpiriGo::setArmed()
{
    mavros_msgs::CommandBool set_armed;
    set_armed.request.value = true;

    if(arm.call(set_armed)){
        ROS_INFO("Set Armed.");
        return set_armed.response.success;
    }else{
        ROS_INFO("Failed to set to Armed.");
        return false;
    }
}

/*******************************************************************************
 *  Make the copter takeoff to a specified height
 *
 *  @param targetAlt a float value of the target altitude (relative to the ground)
 *  @return True if the takeoff command has been successfully issued to the copter
 ******************************************************************************/
void SpiriGo::takeOff(float targetAlt = 5)
{
    // try to take off
    mavros_msgs::CommandTOL to_cmd;

    to_cmd.request.altitude = targetAlt;

    // TODO: make this a bool function based on the success of the call
    if(takeoff.call(to_cmd)){
        ROS_INFO("Taking off");
        taking_off = true;
    }else{
        ROS_INFO("Failed to initiate take off");
    }
}

/*******************************************************************************
 *  Tell the copter to go to specified heading
 *
 *  @param targetYaw angle (in degrees) of the desired heading (from north = 0 degrees)
 *  @param targetYawRate angular velocity in deg/s of the desired yaw rate
 *  @return True if the yaw command has been successfully issued to the copter
 ******************************************************************************/
void SpiriGo::conditionYaw(float targetYaw, float targetYawRate)
{
    mavros_msgs::CommandLong yawCmd;

    yawCmd.request.command = 155;           // MAVLink command ID for MAV_CMD_CONDITION_YAW
    yawCmd.request.confirmation = 0;        // 0 is default for confirmation
    yawCmd.request.param1 = targetYaw;      // target heading/yaw in degrees from north (0 to 360)
    yawCmd.request.param2 = targetYawRate;  // target yaw rate in deg/s
    yawCmd.request.param3 = 1;              // direction; -1 ccw, 1 cw TODO: make this automatic
    yawCmd.request.param4 = 0;              // relative offset 1, absolute angle 0

    // TODO: return true if the call was a success 
    if(mavlink_cmd_srv.call(yawCmd)){
        ROS_INFO("Controlling yaw");
    }else{
        ROS_INFO("Condition yaw command rejected");
    }
}

// TODO: @ssorl what is your plan with this
void SpiriGo::setENUVelocity(/*double eastwardVelocity, double northwardVelocity */)
{
    geometry_msgs::TwistStamped control_msg;

    control_msg.twist.linear.x = velocity.linear.x;
    control_msg.twist.linear.y = velocity.linear.y;

    vel.publish(control_msg);
}

////////////////////////////////////////////////////////////////////////////////
// Action-specific methods - these will sleep until finished!!!
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *  Full autonomous takeoff action
 *
 *  This will ask the copter to go to GUIDED
 *
 *  @param TakeoffGoalConstPtr the takeoff action goal object 
 ******************************************************************************/
void SpiriGo::armAndTakeOff(const spiri_go::TakeoffGoalConstPtr& goal)
{
    ROS_INFO("server excuteCB!");
    if(!takeoff_as.isActive()||takeoff_as.isPreemptRequested()) return;
    ros::Rate takeoff_rate(5);
    bool success = true;

    // Set to guided mode and THEN arm the copter
    bool armed_ = setArmed();
    while(not last_state.armed)
    {
        if(not last_state.guided){
            setGuided();
        }else{
            bool armed_ = setArmed();
        }
        takeoff_rate.sleep();
    }

    // Sleep the method until the takeoff is complete
    taking_off = false;
    while(not taking_off)
    {
        takeOff(goal->height);
        takeoff_rate.sleep();
    }
    while(last_alt.altitude < goal->height*0.96)
    {
        // Only wait to reach 96% of the target altitude to handle undershoot
        takeoff_rate.sleep();
        ROS_INFO("height: %f", last_alt.altitude);
    }
    ROS_INFO("took off");
    flying = true;

    if(ros::ok()){
        takeoff_as.setSucceeded();
        ROS_INFO("Takeoff finished");
    } else {
        takeoff_as.setAborted();
        ROS_INFO("Takeoff aborted");
    }
}

/*******************************************************************************
 *  Full autonomous land action
 *
 *  This will ask the copter to go to LAND mode
 *
 *  @param LandHereGoalConstPtr the land_here action goal object 
 ******************************************************************************/
void SpiriGo::landHere(const spiri_go::LandHereGoalConstPtr& goal)
{
    ROS_INFO("server excuteCB!");
    if(!land_here_as.isActive()||land_here_as.isPreemptRequested()) return;
    ros::Rate land_rate(5);
    bool success = true;

    ROS_INFO("Attempting to land in place");

    // Change the mode to LAND in APM, which is autonomous landing
    while(last_state.mode != "LAND")
    {
        setMode("LAND");
        land_rate.sleep();
    }

    // Sleep until for the robot to think it's 5cm off the ground
    while(location.position.z > 0.05)
    {
        land_rate.sleep();
    }

    if(ros::ok()){
        land_here_as.setSucceeded();
        ROS_INFO("Landing finished");
    } else {
        land_here_as.setAborted();
        ROS_INFO("Landing finished");
    }
}


////////////////////////////////////////////////////////////////////////////////
// Getter functions
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *  @return The latest information on whether the copter is armed
 ******************************************************************************/
bool SpiriGo::isArmed(){
    return last_state.armed;
}

/*******************************************************************************
 *    @return The latest information on whether the copter is in GUIDED mode
 ******************************************************************************/
bool SpiriGo::isControllable(){
    return last_state.guided;
}

/*******************************************************************************
 *    @return The latest information on whether the copter has connected to mavros
 ******************************************************************************/
bool SpiriGo::isValid() {
    return last_connect_state.is_valid;
}

/*******************************************************************************
 *  @return The latest information the mode of the copter
 ******************************************************************************/
std::string SpiriGo::getMode(){
    return last_state.mode;
}


////////////////////////////////////////////////////////////////////////////////
// Utility Methods 
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *  Make a best guess as to the current location
 *  since the real value is provided fairly often
 *  don't need to be too precise about it
 *
 *  @param dt_dur how long since the last update
 ******************************************************************************/
void SpiriGo::updateLocation(ros::Duration dt_dur){
    float dt = dt_dur.toSec();
    location.position.x += velocity.linear.x*dt;
    location.position.y += velocity.linear.y*dt;
    location.position.z += velocity.linear.z*dt;
    // TODO: deal with the angular part too
}

////////////////////////////////////////////////////////////////////////////////
// Main loop
////////////////////////////////////////////////////////////////////////////////

void SpiriGo::Loop()
{
    ros::Rate pub_rate(10);

    ros::Time last_time = ros::Time::now();
    while (nh.ok()){
        ros::spinOnce();
        if(flying){
            ros::Duration dt =  ros::Time::now() - last_time;
            last_time = ros::Time::now();
            //updateLocation(dt);
            //setENUVelocity();
        }

        pub_rate.sleep();
    }

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "spiri_go");

    SpiriGo go_thing;

    //wait until apm has connected to mavros
    /*while(!go_thing.isValid()) {
        ;
    }
    ROS_INFO("connected to mavros");*/
    //I found that the nodes not run in order has no effect

    //It's better to wait some time
    ros::Duration(2).sleep();

    //change stream rate, important
    while(!go_thing.setStreamRate(0, 10)) {   //-all  
    }

    ROS_INFO("set action server!");

    // start action servers
    go_thing.takeoff_as.start();
    go_thing.land_here_as.start();

    go_thing.Loop();

    return 0;
}
