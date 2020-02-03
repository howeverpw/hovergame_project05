/*
*   HoverGames Challenge 1: Fight Fire with Flyers
*
*   Project: Air Strategist Companion
*   Author: Raul Alvarez-Torrico (raul@tecbolivia.com)
*   Date: 02/02/2020
*
*   @file mavros_offboard_node.cpp
*   @brief Controls autonomously the drone's flight by using PX4's "offboard" flight mode
*   
*   The "offboard" flying part of this code is based on the offb_node.cpp MAVROS example
*/

// TODO: Some of the following includes, defines and variables might be unused
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/console.h>

#include <iostream>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/WaypointList.h>
#include <std_msgs/Int16.h>
#include <mavros_msgs/VFR_HUD.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_datatypes.h>

#include <iomanip>      // For std::setprecision

#include <ctime>
#include <ratio>
#include <chrono>
#include <cstdlib> // For abs()

using namespace std::chrono;

#define DES_START_ALTITUDE  2.0
#define DES_YAW_ANGLE_DEG   90.0 // 90* is North
#define MAX_XY_RAW_VELOCITY 4.0 //1.0
#define OFFBOARD_FLYING_ALTITUDE    10
#define DISTANCE_SCALING_FACTOR 0.3 // JUST FOR DEBUGGING
#define TIME_DELAY_AT_GOAL 5000
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define radiansToDegrees(angleRadians) ((angleRadians) * 180.0 / M_PI)


// This would be the same as sel_crew_member_idx from another topic
int fighter_crew_list_current_seq = 1; 
mavros_msgs::WaypointList fighter_crew_list;
void fighter_crew_list_callback(const mavros_msgs::WaypointList& msg) {
    // std::cout << "fighter_crew_list_callback msg: " << msg << std::endl;
    // std::cout << "msg.current_seq: " << msg.current_seq << std::endl;
    // std::cout << "msg.waypoints[0].x_lat: " << msg.waypoints[0].x_lat << std::endl;
    // std::cout << "msg.waypoints[1].x_lat: " << msg.waypoints[1].x_lat << std::endl;
    // std::cout << "msg.waypoints[2].x_lat: " << msg.waypoints[2].x_lat << std::endl;
    fighter_crew_list_current_seq = msg.current_seq; // This would be the same as sel_crew_member_idx from another topic

    fighter_crew_list = msg;
}

// 
mavros_msgs::State drone_current_state;
void state_callback(const mavros_msgs::State::ConstPtr& msg){
    drone_current_state = *msg;
}

float received_cmd_vel_linear_x, received_cmd_vel_linear_y, received_cmd_vel_linear_z;
float received_cmd_vel_angular_x, received_cmd_vel_angular_y, received_cmd_vel_angular_z;
void cmd_vel_callback(const geometry_msgs::Twist& msg){

    received_cmd_vel_linear_x = msg.linear.x;
    received_cmd_vel_linear_y = msg.linear.y;
    received_cmd_vel_linear_z = msg.linear.z;

    received_cmd_vel_angular_x = msg.angular.x;
    received_cmd_vel_angular_y = msg.angular.y;
    received_cmd_vel_angular_z = msg.angular.z;

//    std::cout << "Linear x: " << received_cmd_vel_linear_x << std::endl;
    std::cout << "Linear vel: " << msg.linear << std::endl;
    std::cout << "Angular vel: " << msg.angular << std::endl;
}

float drone_position_x, drone_position_y, drone_position_z;
geometry_msgs::Quaternion drone_orientation_quat;
void drone_pose_callback(const geometry_msgs::PoseStamped& msg){

    drone_position_x = msg.pose.position.x;
    drone_position_y = msg.pose.position.y;
    drone_position_z = msg.pose.position.z;

    // drone_orientation_quat = msg.pose.orientation;
//    std::cout << "Position z: " << drone_position_z << std::endl;
}

float global_drone_position_latitude, global_drone_position_longitude, global_drone_position_altitude;
void drone_global_pose_callback(const sensor_msgs::NavSatFix& msg) {
    global_drone_position_latitude = msg.latitude;
    global_drone_position_longitude = msg.longitude;
}

float received_fire_pose_x, received_fire_pose_y, received_fire_pose_z;
void fire_pose_callback(const geometry_msgs::Point& msg){

    received_fire_pose_x = msg.x;
    received_fire_pose_y = msg.y;
    received_fire_pose_z = msg.z;

    // std::cout << "Fire pose x: " << received_fire_pose_x << std::endl;
    // std::cout << "Fire pose: " << msg << std::endl;
}

// Callback for receiving Fire Spot 1 pose point
float received_fire_spot1_pose_x, received_fire_spot1_pose_y, received_fire_spot1_pose_z;
void fire_spot1_pose_callback(const geometry_msgs::Point& msg){

    received_fire_spot1_pose_x = msg.x;
    received_fire_spot1_pose_y = msg.y;
    received_fire_spot1_pose_z = msg.z;

    // std::cout << "Fire spot1_pose x: " << received_fire_spot1_pose_x << std::endl;
    // std::cout << "Fire pose: " << msg << std::endl;
}

// Callback for receiving Fire Spot 2 pose point
float received_fire_spot2_pose_x, received_fire_spot2_pose_y, received_fire_spot2_pose_z;
void fire_spot2_pose_callback(const geometry_msgs::Point& msg){

    received_fire_spot2_pose_x = msg.x;
    received_fire_spot2_pose_y = msg.y;
    received_fire_spot1_pose_z = msg.z;

    // std::cout << "Fire spot2_pose x: " << received_fire_spot2_pose_x << std::endl;
    // std::cout << "Fire pose: " << msg << std::endl;
}

// Callback for receiving Fire Spot 3 pose point
float received_fire_spot3_pose_x, received_fire_spot3_pose_y, received_fire_spot3_pose_z;
void fire_spot3_pose_callback(const geometry_msgs::Point& msg){

    received_fire_spot3_pose_x = msg.x;
    received_fire_spot3_pose_y = msg.y;
    received_fire_spot3_pose_z = msg.z;

    // std::cout << "Fire spot3_pose x: " << received_fire_spot3_pose_x << std::endl;
    // std::cout << "Fire pose: " << msg << std::endl;
}

// This is just for debugging:
void vfr_hud_callback(const mavros_msgs::VFR_HUD& msg){
    // std::cout << "vfr_hud_callback called..." << std::endl;
}

float pose_diff_x;
float pose_diff_y;
void fire_pose_euclidean(float drone_pos_x, float drone_pos_y, float fire_pos_x, float fire_pos_y) {
    pose_diff_x = fire_pos_x - drone_pos_x;
    pose_diff_y = fire_pos_y - drone_pos_y;
}

float clip_velocity(float value, float minimum, float maximum) {
    //"""Ensure value is between minimum and maximum."""

    if(value < minimum) {
        return minimum;
    }
    else if(value > maximum) {
        return maximum;
    }
    else {
        return value;
    }
}

int task_mode_cmd = 1; // Default
int last_task_mode_cmd = 0; // Default
float target_pose_x, target_pose_y, target_pose_z;

enum drone_tasks {
    TRACK_PATTERN = 1,
    TRACK_FIRE_SPOT_1,
    TRACK_FIRE_SPOT_2,
    TRACK_FIRE_SPOT_3,
    TRACK_ALL_SPOTS,
    TRACK_CREW_MEMBER,
    TRACK_ALL_CREW_MEMBERS
};

void drone_commands_callback(const std_msgs::Int16& msg){
    task_mode_cmd = msg.data;
    // std::cout << "[OFFBOARD NODE] drone_commands_callback task_mode_cmd: " << task_mode_cmd<< std::endl;
}

int sel_crew_member_idx = 1; // Default
void sel_crew_member_idx_callback(const std_msgs::Int16& msg) {
    sel_crew_member_idx = msg.data;
}

#include <math.h>
#include <cmath> 
#define earthRadiusKm 6371.0

// This function converts decimal degrees to radians
double deg2rad(double deg) {
  return (deg * M_PI / 180.0);
}

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
  return (rad * 180.0 / M_PI);
}

/**
 * Returns the distance between two points on the Earth.
 * Direct translation from http://en.wikipedia.org/wiki/Haversine_formula
 * @param lat1d Latitude of the first point in degrees
 * @param lon1d Longitude of the first point in degrees
 * @param lat2d Latitude of the second point in degrees
 * @param lon2d Longitude of the second point in degrees
 * @return The distance between the two points in kilometers
 */
double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) {
  double lat1r, lon1r, lat2r, lon2r, u, v;
  lat1r = deg2rad(lat1d);
  lon1r = deg2rad(lon1d);
  lat2r = deg2rad(lat2d);
  lon2r = deg2rad(lon2d);
  u = sin((lat2r - lat1r)/2.0);
  v = sin((lon2r - lon1r)/2.0);
  // return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v)); // return in Km
  return 1000.0 * 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v)); // return in m
}

double azimuth(double gps_lat, double gps_lon, double cur_waypoint_lat, double cur_waypoint_lon) {
    // Compute the forward azimuth
  // **************************************************
  double gps_f_lat_rad = deg2rad(gps_lat);
  double gps_f_lon_rad = deg2rad(gps_lon);
  double waypoint_lat_rad = deg2rad(cur_waypoint_lat);
  double waypoint_lon_rad = deg2rad(cur_waypoint_lon);

  double waypoint_angle = atan2(sin(waypoint_lon_rad - gps_f_lon_rad) * cos(waypoint_lat_rad), 
                cos(gps_f_lat_rad) * sin(waypoint_lat_rad) - sin(gps_f_lat_rad) * cos(waypoint_lat_rad) * cos(waypoint_lon_rad - gps_f_lon_rad));
  
  // waypoint_angle = waypoint_angle * 180.0 / M_PI; // Convert from radians to degrees

  // Always convert to positive angles
  // if (waypoint_angle < 0) {
  //   waypoint_angle += 360.0;
  // }

  return waypoint_angle;
}

double azimuth2angle(double azimuth) {
    //subtract the bearing angle from 90°. If you end up with a negative answer, add 360°, 
    //and if your answer is greater than 360°, subtract 360° from it.
    // double angle_to_fighter = 90 - azimuth_to_fighter;

    double angle = M_PI/2.0 - azimuth;
    if(angle < 0) { angle += 2 * M_PI; }
    else if(angle > (2 * M_PI)) { angle -= 2 * M_PI; }

    return angle;
}



bool desired_altitude_reached = false;
double prev_target_pose_x = 0;
double prev_target_pose_y = 0;
double lastErr_x = 0;
double lastErr_y = 0;
double errSum_x = 0;
double errSum_y = 0;
// long int lastTime;

high_resolution_clock::time_point lastTime = high_resolution_clock::now();

bool is_going_to_crew_member = false;
bool is_going_to_fire_spot = false;
bool is_going_to_target_pose = false;
high_resolution_clock::time_point last_tracking_time = high_resolution_clock::now();

unsigned int fighter_crew_list_size;
unsigned short int fire_spots_list_size;
unsigned short int fighter_tracking_index = 0;
unsigned short int fire_spot_tracking_index = 0;

std::vector<std::string> available_task_mode_cmds = {"TRACK_PATTERN", "TRACK_FIRE_SPOT_1", "TRACK_FIRE_SPOT_2",
                                    "TRACK_FIRE_SPOT_3", "TRACK_ALL_SPOTS", "TRACK_CREW_MEMBER", "TRACK_ALL_CREW_MEMBERS"}; 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    std::cout << "[OFFBOARD NODE] Starting... " << std::endl;

    // This is just a memory help:
    // ROS_INFO("ROS_INFO");
    // ROS_WARN("ROS_WARN");
    // ROS_ERROR("ROS_ERROR");
    // std::cout << "cout" << std::endl;
    // std::cerr << "cerr" << std::endl;

    ros::NodeHandle nh;

    //// -------------------- Create ROS subscribers --------------------
    // To receive the drone's current state
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_callback);

    // JUST FOR DEBUGGING: To control the drone from the computer keyboard
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmd_vel_callback);

    // To receive the drone's current local position
    ros::Subscriber pose_sub = nh.subscribe("/mavros/local_position/pose", 10, drone_pose_callback);

    // To receive the drone's global position
    ros::Subscriber global_pose_sub = nh.subscribe("/mavros/global_position/global", 10, drone_global_pose_callback);

    // To receive the the fire pattern centroid's local Cartesian coordinates
    ros::Subscriber fire_pose_sub = nh.subscribe("/fire_pattern/pose_point", 10, fire_pose_callback);

    // To receive the local Cartesian coordinates of Fire Spots 1, 2, 3
    ros::Subscriber fire_spot1_pose_sub = nh.subscribe("/fire_pattern/fire_spot1_pose_point", 10, fire_spot1_pose_callback);
    ros::Subscriber fire_spot2_pose_sub = nh.subscribe("/fire_pattern/fire_spot2_pose_point", 10, fire_spot2_pose_callback);
    ros::Subscriber fire_spot3_pose_sub = nh.subscribe("/fire_pattern/fire_spot3_pose_point", 10, fire_spot3_pose_callback);

    // To receive the task mode commands received from the web page's control pane
    ros::Subscriber drone_commands_sub = nh.subscribe("/drone_commands/task_mode_cmd", 10, drone_commands_callback);

    // To receive the index of the current selected firefighter to be tracked
    // received from the web page's control pane
    ros::Subscriber sel_crew_member_idx_sub = nh.subscribe("/drone_commands/sel_crew_member_idx", 10, sel_crew_member_idx_callback);

    // To receive the coordinates of all firefighters currently detected by the sistem
    // received from the web page's control pane
    ros::Subscriber fighter_crew_list_sub = nh.subscribe("/drone_commands/fighter_crew_list", 10, fighter_crew_list_callback);
    
    // JUST FOR DEBUGGING: For getting the drone's groundspeed
    // ros::Subscriber vfr_hud_sub = nh.subscribe("/mavros/vfr_hud", 10, vfr_hud_callback);

    //// -------------------- Create ROS publishers ------------------
    // For publishing the coordinates the drone must track
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // --------------------------------------------------------------------------------
    // JUST FOR DEBUGGING: The following publishers are just for trying other types of
    // "offboard" control that could be useful in the future. Not currently used
    // To send 
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
     ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    // For publishing setpoint_raw: /mavros/setpoint_raw/local

    // rostopic pub /mavros/setpoint_raw/local mavros_msgs/PositionTarget
    // '{header: {stamp: now, frame_id: "world"}, coordinate_frame: 8, type_mask: 3527,
    // velocity: {x: 0.1, y: 0, z: 0}}' -r 10
    ros::Publisher setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
     ("/mavros/setpoint_raw/local", 10);

    // Publish global position for following firefighters
    ros::Publisher global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("/mavros/setpoint_raw/global", 10);
    ros::Publisher geo_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>
            ("/mavros/setpoint_position/global", 10);
    // --------------------------------------------------------------------------------

    //// -------------------- Create ROS services ------------------
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !drone_current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("FCU connection OK");
    // std::cout << "FCU connection OK" << std::endl;

    // geometry_msgs/Point position | geometry_msgs/Quaternion orientation
    // Even though the PX4 Pro Flight Stack operates in the aerospace NED coordinate frame, 
    // MAVROS translates these coordinates to the standard ENU frame and vice-versa. 
    // This is why we set z to positive 2.
    geometry_msgs::PoseStamped desired_pose_msg;
    desired_pose_msg.pose.position.x = 0;
    desired_pose_msg.pose.position.y = 0;
    desired_pose_msg.pose.position.z = DES_START_ALTITUDE;

    // Our desired angles:
    tf::Quaternion mav_orientation = tf::createQuaternionFromRPY(0.0, 0.0, degreesToRadians(DES_YAW_ANGLE_DEG));

    desired_pose_msg.pose.orientation.x = mav_orientation.x();
    desired_pose_msg.pose.orientation.y = mav_orientation.y();
    desired_pose_msg.pose.orientation.z = mav_orientation.z();
    desired_pose_msg.pose.orientation.w = mav_orientation.w();

    // -------------------------------------------------------------------------------
    // JUST FOR DEBUGGIN: The following messages are just for debugging. They could be
    // useful in the future, not currently inplemented
    geometry_msgs::Twist local_vel_msg;
    local_vel_msg.linear.x = 0.0;
    local_vel_msg.linear.y = 0.0;
    local_vel_msg.linear.z = 0.0;
    local_vel_msg.angular.x = 0.0;
    local_vel_msg.angular.y = 0.0;
    local_vel_msg.angular.z = 0.0;

    // ---- Create PositionTarget message object to track fire spots
    mavros_msgs::PositionTarget raw_velocity_msg;
    raw_velocity_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    raw_velocity_msg.header.frame_id = "drone";
    raw_velocity_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                             mavros_msgs::PositionTarget::IGNORE_PY |
                             // mavros_msgs::PositionTarget::IGNORE_PZ |
                             mavros_msgs::PositionTarget::IGNORE_AFX |
                             mavros_msgs::PositionTarget::IGNORE_AFY |
                             mavros_msgs::PositionTarget::IGNORE_AFZ |
                             mavros_msgs::PositionTarget::FORCE |
                             // mavros_msgs::PositionTarget::IGNORE_YAW |
                             mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;
    raw_velocity_msg.header.stamp = ros::Time::now();
    raw_velocity_msg.velocity.x = 0;
    raw_velocity_msg.velocity.y = 0;
    raw_velocity_msg.velocity.z = 0;

    geographic_msgs::GeoPoseStamped geo_pose_stamped_msg;
    geo_pose_stamped_msg.pose.position.latitude = 0;
    geo_pose_stamped_msg.pose.position.longitude = 0;
    geo_pose_stamped_msg.pose.position.altitude = OFFBOARD_FLYING_ALTITUDE;

    // ---- Create GlobalPositionTarget message object to track firefighter crew members
    mavros_msgs::GlobalPositionTarget global_pos_msg;
    // global_pos_msg.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    global_pos_msg.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    global_pos_msg.header.frame_id = "drone";
    global_pos_msg.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX |
                                mavros_msgs::GlobalPositionTarget::IGNORE_VY |
                                mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
                                mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                                mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
                                mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                                // mavros_msgs::GlobalPositionTarget::FORCE |
                                // mavros_msgs::GlobalPositionTarget::IGNORE_YAW |
                                mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE ;
    global_pos_msg.header.stamp = ros::Time::now();
    global_pos_msg.latitude = 0;
    global_pos_msg.longitude = 0;
    global_pos_msg.altitude = OFFBOARD_FLYING_ALTITUDE;
    // global_pos_msg.velocity = []; // geometry_msgs/Vector3 velocity
    // global_pos_msg.acceleration_or_force = [];// geometry_msgs/Vector3 acceleration_or_force
    global_pos_msg.yaw = 0;
    global_pos_msg.yaw_rate = 0;
    // -------------------------------------------------------------------------------

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(desired_pose_msg);
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        // ---------------- Bypass the automatic "offboard" mode ----------------
        // if( drone_current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !drone_current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }
        // END: ---------------- Bypass the automatic "offboard" mode ----------------

        // ---------------- Bypass the minumum atarting altitude verification ----------------
        // // Drive the drone to a desired altitude
        // if(drone_position_z < 2.0*0.95 && !desired_altitude_reached) {
        //     local_pos_pub.publish(desired_pose_msg);
        // }
        // // Change flag to reached desired altitude
        // else if(!desired_altitude_reached){ 
        //     desired_altitude_reached = true;
        //     ROS_INFO("desired_altitude_reached ...");
        //     // std::cout << "desired_altitude_reached ..." << std::endl;
        //     mav_orientation = tf::createQuaternionFromRPY(0.0, 0.0, degreesToRadians(DES_YAW_ANGLE_DEG));
        // }

        desired_altitude_reached = true;
        // ---------------- Bypass the minumum atarting altitude verification ----------------

        // ---------------- Bypass the minumum atarting altitude verification ----------------
        // -------- JUST FOR DEBUGGING: Test the desired velocity with local_vel_msg -------- 
        // local_vel_msg.linear.x = received_cmd_vel_linear_x;
        // local_vel_msg.linear.y = received_cmd_vel_linear_y;
        // //local_vel_msg.linear.y = received_cmd_vel_angular_z;
        // local_vel_msg.linear.z = received_cmd_vel_linear_z;

        // local_vel_msg.angular.z = received_cmd_vel_angular_z;

        // local_vel_pub.publish(local_vel_msg);
        // -------- JUST FOR DEBUGGING: Test the desired velocity with local_vel_msg -------- 


        // ---------------- Set selected target from web page ----------------
        // std::cout << "[OFFBOARD NODE] task_mode_cmd: " << task_mode_cmd<< std::endl;

        /*
        * **********************************************************************************
        * TODO: This part of the code evaluates the different task mode commands the drone
        * receives and decides which action to take.
        * It's not pretty, it's convoluted, it's a work in progress. I kind of made a mess
        * of it when implemmenting the two "patrolling" modes.
        * Sadly, didn't have enough time to make as it should be before the contest 
        * deadline, but works.
        * **********************************************************************************
        */

        // If the task mode command has changed:
        if(task_mode_cmd != last_task_mode_cmd) {
            std::cout << "[OFFBOARD NODE] new task_mode_cmd is: " << available_task_mode_cmds[task_mode_cmd - 1] << std::endl;
            last_task_mode_cmd = task_mode_cmd;

            std::cout << "[OFFBOARD NODE] is_going_to_fire_spot=" << is_going_to_fire_spot
                        << " | is_going_to_target_pose=" << is_going_to_target_pose
                        << " | is_going_to_crew_member=" << is_going_to_crew_member
                        << std::endl;
        }
        switch(task_mode_cmd) {
            // In the following 4 modes, just assign to target_pose the corresponding
            // local coordinates for the current target
            case TRACK_PATTERN:
                target_pose_x = received_fire_pose_x * DISTANCE_SCALING_FACTOR;
                target_pose_y = received_fire_pose_y * DISTANCE_SCALING_FACTOR;
            break;

            case TRACK_FIRE_SPOT_1:
                target_pose_x = received_fire_spot1_pose_x * DISTANCE_SCALING_FACTOR; 
                target_pose_y = received_fire_spot1_pose_y * DISTANCE_SCALING_FACTOR;
            break;

            case TRACK_FIRE_SPOT_2:
                target_pose_x = received_fire_spot2_pose_x * DISTANCE_SCALING_FACTOR; 
                target_pose_y = received_fire_spot2_pose_y * DISTANCE_SCALING_FACTOR;
            break;

            case TRACK_FIRE_SPOT_3:
                target_pose_x = received_fire_spot3_pose_x * DISTANCE_SCALING_FACTOR; 
                target_pose_y = received_fire_spot3_pose_y * DISTANCE_SCALING_FACTOR;
            break;

            case TRACK_ALL_SPOTS:
                is_going_to_crew_member = false;
                is_going_to_target_pose = false;
                // std::cout << "[OFFBOARD NODE] task_mode_cmd: " << task_mode_cmd<< std::endl;
                fire_spots_list_size = 3;
                // std::cout << "[OFFBOARD NODE] fighter_crew_list_size: " << fighter_crew_list_size << std::endl;

                { // I don't know why, but doesn't compile without this pair brackets!
                    double fire_error_pose_x;
                    double fire_error_pose_y;
                    double cur_fire_spot_pose_x;
                    double cur_fire_spot_pose_y;

                    // Check which fire spot to track and calculate the error
                    switch(fire_spot_tracking_index) {
                        case 0:
                            fire_error_pose_x = received_fire_spot1_pose_x * DISTANCE_SCALING_FACTOR - drone_position_x;
                            fire_error_pose_y = received_fire_spot1_pose_y * DISTANCE_SCALING_FACTOR - drone_position_y;
                        break;

                        case 1:
                            fire_error_pose_x = received_fire_spot2_pose_x * DISTANCE_SCALING_FACTOR - drone_position_x;
                            fire_error_pose_y = received_fire_spot2_pose_y * DISTANCE_SCALING_FACTOR - drone_position_y;
                        break;

                        case 2:
                            fire_error_pose_x = received_fire_spot3_pose_x * DISTANCE_SCALING_FACTOR - drone_position_x;
                            fire_error_pose_y = received_fire_spot3_pose_y * DISTANCE_SCALING_FACTOR - drone_position_y;
                        break;
                    }

                    // std::cout << "[OFFBOARD NODE] fire_error_pose_x=" << fire_error_pose_x << std::endl;

                    // Compute the Euclidean distance to the goal
                    double euclidean_error = sqrt(pow(fire_error_pose_x, 2) + pow(fire_error_pose_y, 2));

                    // The drone isn't at goal position && is_going_to_fire_spot = false
                    if( (euclidean_error > 1) && !is_going_to_fire_spot) { 
                        // Compute new absolute_local_pose and set it
                        std::cout << "[OFFBOARD NODE] fire_spot_tracking_index: " << fire_spot_tracking_index << std::endl;
                        // std::cout << "[OFFBOARD NODE] fighter_latitude=" << fighter_latitude
                        //         << " | fighter_longitude=" << fighter_longitude << std::endl;

                        // std::cout << "[OFFBOARD NODE] distance_to_fighter=" << distance_to_fighter 
                        //         << " | azimuth_to_fighter=" << azimuth_to_fighter << " | angle_to_fighter=" << angle_to_fighter << std::endl;
                        std::cout << "[OFFBOARD NODE] drone_position_x=" << drone_position_x
                                << " | drone_position_y=" << drone_position_y 
                                << " | fire_error_pose_x=" << fire_error_pose_x << std::endl;


                        desired_pose_msg.pose.position.x = drone_position_x + fire_error_pose_x;
                        desired_pose_msg.pose.position.y = drone_position_y + fire_error_pose_y;
                        desired_pose_msg.pose.position.z = OFFBOARD_FLYING_ALTITUDE;

                        is_going_to_fire_spot = true; // Flag the drone is_going_to_fire_spot
                    }
                    // The drone is at goal position
                    else if (euclidean_error <= 1) { 

                        if(is_going_to_fire_spot) { // is_going_to_fire_spot & the drone just reached the goal
                            std::cout << "[OFFBOARD NODE] >>> new goal reached!" << " | fire_error_pose_x=" << fire_error_pose_x << std::endl;
                            std::cout << "[OFFBOARD NODE] ...tracking delay started..." << std::endl;

                            // Start the time delay
                            last_tracking_time = high_resolution_clock::now();
                        }

                        // Check if the time delay is over
                        high_resolution_clock::time_point now = high_resolution_clock::now();
                        duration<double, std::milli> time_span = now - last_tracking_time;

                        if(time_span.count() >= TIME_DELAY_AT_GOAL) {
                            std::cout << "[OFFBOARD NODE] ...tracking delay finished..." << std::endl;

                            // Change index to the next spot to track
                            if(fire_spot_tracking_index < (fire_spots_list_size - 1)) {
                                ++fire_spot_tracking_index;
                            }
                            else {
                                fire_spot_tracking_index = 0;
                            }
                        }

                        is_going_to_fire_spot = false; // The drone reached the goal

                    }
                }
            break; // END case TRACK_ALL_SPOTS

            case TRACK_ALL_CREW_MEMBERS:
                is_going_to_fire_spot = false;
                is_going_to_target_pose = false;
                // std::cout << "[OFFBOARD NODE] is_going_to_crew_member: " << is_going_to_crew_member<< std::endl;
                fighter_crew_list_size = fighter_crew_list.waypoints.size();
                // std::cout << "[OFFBOARD NODE] fighter_crew_list_size: " << fighter_crew_list_size << std::endl;

                if(fighter_crew_list_size > 0) {
                    // std::cout << "[OFFBOARD NODE] task_mode_cmd: " << task_mode_cmd<< std::endl;
                    // std::cout << "[OFFBOARD NODE] fighter_crew_list_size: " << fighter_crew_list_size << std::endl;
                    // 
                    double fighter_latitude = fighter_crew_list.waypoints[fighter_tracking_index].x_lat;
                    double fighter_longitude = fighter_crew_list.waypoints[fighter_tracking_index].y_long;
                    double distance_to_fighter = distanceEarth(global_drone_position_latitude, global_drone_position_longitude, 
                                fighter_latitude, fighter_longitude);
                    double azimuth_to_fighter = azimuth(global_drone_position_latitude, global_drone_position_longitude, 
                                fighter_latitude, fighter_longitude);

                    double angle_to_fighter = azimuth2angle(azimuth_to_fighter);

                    double member_error_pose_x = distance_to_fighter * cos(angle_to_fighter);
                    double member_error_pose_y = distance_to_fighter * sin(angle_to_fighter);

                    // std::cout << "[OFFBOARD NODE] member_error_pose_x=" << member_error_pose_x << std::endl;

                    // Compute the Euclidean distance to the goal
                    double euclidean_error = sqrt(pow(member_error_pose_x, 2) + pow(member_error_pose_y, 2));

                    // The drone isn't at goal position && is_going_to_crew_member = false
                    if( (euclidean_error > 1) && !is_going_to_crew_member) { 
                        // Compute new absolute_local_pose and set it
                        std::cout << "[OFFBOARD NODE] fighter_tracking_index: " << fighter_tracking_index << std::endl;
                        std::cout << "[OFFBOARD NODE] fighter_latitude=" << fighter_latitude
                                << " | fighter_longitude=" << fighter_longitude << std::endl;

                        std::cout << "[OFFBOARD NODE] distance_to_fighter=" << distance_to_fighter 
                                << " | azimuth_to_fighter=" << azimuth_to_fighter << " | angle_to_fighter=" << angle_to_fighter << std::endl;
                        std::cout << "[OFFBOARD NODE] drone_position_x=" << drone_position_x
                                << " | drone_position_y=" << drone_position_y 
                                << " | member_error_pose_x=" << member_error_pose_x << std::endl;


                        desired_pose_msg.pose.position.x = drone_position_x + member_error_pose_x;
                        desired_pose_msg.pose.position.y = drone_position_y + member_error_pose_y;
                        desired_pose_msg.pose.position.z = OFFBOARD_FLYING_ALTITUDE;

                        is_going_to_crew_member = true; // Flag the drone is_going_to_crew_member
                    }
                    // The drone is at goal position
                    else if (euclidean_error <= 1) { 

                        if(is_going_to_crew_member) { // is_going_to_crew_member & the drone just reached the goal
                            std::cout << "[OFFBOARD NODE] >>> new goal reached!" << " | member_error_pose_x=" << member_error_pose_x << std::endl;
                            std::cout << "[OFFBOARD NODE] ...tracking delay started..." << std::endl;

                            // Start the time delay
                            last_tracking_time = high_resolution_clock::now();
                        }

                        // Check if the time delay is over
                        high_resolution_clock::time_point now = high_resolution_clock::now();
                        duration<double, std::milli> time_span = now - last_tracking_time;

                        if(time_span.count() >= TIME_DELAY_AT_GOAL) {
                            std::cout << "[OFFBOARD NODE] ...tracking delay finished..." << std::endl;

                            // Change index to the next firefighter to track
                            if(fighter_tracking_index < (fighter_crew_list_size - 1)) {
                                ++fighter_tracking_index;
                            }
                            else {
                                fighter_tracking_index = 0;
                            }
                        }

                        is_going_to_crew_member = false; // The drone reached the goal

                    }
                }
                else {
                    std::cout << "[OFFBOARD NODE] fighter_crew_list_size is 0! " << std::endl;
                }

            break; // END case TRACK_ALL_CREW_MEMBERS:

            case TRACK_CREW_MEMBER:
                is_going_to_fire_spot = false;
                is_going_to_target_pose = false;
                // The first time(s) fighter_crew_list.current_seq is zero, but I can't catch the error!
                // Perhaps because I'm trying to acces the object before it's populated for the first time
                // that's why I put the 'if' statement
                
                fighter_crew_list_size = fighter_crew_list.waypoints.size();
                // std::cout << "[OFFBOARD NODE] fighter_crew_list_size: " << fighter_crew_list_size << std::endl;

                if(fighter_crew_list_size > 0 && fighter_crew_list.current_seq > 0) {
                    double fighter_latitude = fighter_crew_list.waypoints[fighter_crew_list.current_seq - 1].x_lat;
                    double fighter_longitude = fighter_crew_list.waypoints[fighter_crew_list.current_seq - 1].y_long;
                    double distance_to_fighter = distanceEarth(global_drone_position_latitude, global_drone_position_longitude, 
                                fighter_latitude, fighter_longitude);
                    double azimuth_to_fighter = azimuth(global_drone_position_latitude, global_drone_position_longitude, 
                                fighter_latitude, fighter_longitude);

                    double angle_to_fighter = azimuth2angle(azimuth_to_fighter);

                    double error_pose_x = distance_to_fighter * cos(angle_to_fighter);
                    double error_pose_y = distance_to_fighter * sin(angle_to_fighter);

                    // Compute the Euclidean distance to the goal
                    double euclidean_error = sqrt(pow(error_pose_x, 2) + pow(error_pose_y, 2));

                    // The drone isn't at goal position && is_going_to_crew_member = false
                    if( (euclidean_error > 1) && !is_going_to_crew_member) { 
                        // Compute new absolute_local_pose and set it
                        std::cout << "[OFFBOARD NODE] distance_to_fighter=" << distance_to_fighter 
                                << " | azimuth_to_fighter=" << azimuth_to_fighter << " | angle_to_fighter=" << angle_to_fighter << std::endl;
                        std::cout << "[OFFBOARD NODE] drone_position_x=" << drone_position_x
                                << " | drone_position_y=" << drone_position_y 
                                << " | error_pose_x=" << error_pose_x << std::endl;

                        desired_pose_msg.pose.position.x = drone_position_x + error_pose_x;
                        desired_pose_msg.pose.position.y = drone_position_y + error_pose_y;
                        desired_pose_msg.pose.position.z = OFFBOARD_FLYING_ALTITUDE;

                        is_going_to_crew_member = true; // Flag the drone is_going_to_crew_member
                    }
                    // The drone is at goal position
                    else if (euclidean_error <= 1) { 
                        if(is_going_to_crew_member) {
                            std::cout << "[OFFBOARD NODE] >>> new goal reached!" << " | error_pose_x=" << error_pose_x << std::endl;

                            // Start the time delay
                            // last_tracking_time = high_resolution_clock::now();
                        }
                        is_going_to_crew_member = false; // The drone reached the goal
                    }
                }
                else {
                    if (fighter_crew_list_size > 0 && fighter_crew_list.current_seq == 0) {
                        std::cout << "[OFFBOARD NODE] fighter_crew_list.current_seq is 0!, select a crew member from the web page." << std::endl;
                    }
                    else {
                        std::cout << "[OFFBOARD NODE] fighter_crew_list_size | fighter_crew_list.current_seq are 0!" << std::endl;
                    }
                }

            break; // END case TRACK_CREW_MEMBER:

            // When the command is unknown or not implemented, track the whole fire pattern
            default:
                target_pose_x = received_fire_pose_x * DISTANCE_SCALING_FACTOR;
                target_pose_y = received_fire_pose_y * DISTANCE_SCALING_FACTOR;
                target_pose_z = received_fire_pose_z;
            break;
        }

        // ---- Update the desired velocity raw:
        if(desired_altitude_reached == true) {
            if(task_mode_cmd == TRACK_CREW_MEMBER 
                || task_mode_cmd == TRACK_ALL_CREW_MEMBERS || task_mode_cmd == TRACK_ALL_SPOTS) {

                local_pos_pub.publish(desired_pose_msg);
            }

            // For task mode cmds: TRACK_PATTERN, TRACK_FIRE_SPOT_1, TRACK_FIRE_SPOT_2, TRACK_FIRE_SPOT_3
            else {
                is_going_to_crew_member = false;
                is_going_to_fire_spot = false;

                double error_pose_x = target_pose_x - drone_position_x;
                double error_pose_y = target_pose_y - drone_position_y;

                // std::cout << "[OFFBOARD NODE] target_error_pose_x=" << error_pose_x << std::endl;

                // Compute the Euclidean distance to the goal
                double euclidean_error = sqrt(pow(error_pose_x, 2) + pow(error_pose_y, 2));

                // The drone isn't at goal position && is_going_to_target_pose = false
                if( (euclidean_error  > 1) && !is_going_to_target_pose) { 
                    // Compute new absolute_local_pose and set it
                    // std::cout << "[OFFBOARD NODE] distance_to_fighter=" << distance_to_fighter 
                    //         << " | azimuth_to_fighter=" << azimuth_to_fighter << " | angle_to_fighter=" << angle_to_fighter << std::endl;
                    std::cout << "[OFFBOARD NODE] drone_position_x=" << drone_position_x
                            << " | drone_position_y=" << drone_position_y 
                            << " | error_pose_x=" << error_pose_x << std::endl;

                    desired_pose_msg.pose.position.x = drone_position_x + error_pose_x;
                    desired_pose_msg.pose.position.y = drone_position_y + error_pose_y;
                    desired_pose_msg.pose.position.z = OFFBOARD_FLYING_ALTITUDE;

                    is_going_to_target_pose = true; // Flag the drone is_going_to_target_pose
                }
                // The drone is at goal position
                else if (euclidean_error  <= 1) { 
                    if(is_going_to_target_pose) {
                        std::cout << "[OFFBOARD NODE] >>> new goal reached!" << " | error_pose_x=" << error_pose_x << std::endl;
                        // last_tracking_time = high_resolution_clock::now();
                    }
                    is_going_to_target_pose = false; // The drone reached the goal

                }

                local_pos_pub.publish(desired_pose_msg);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

