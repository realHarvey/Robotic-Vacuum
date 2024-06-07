// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// webots_ros
#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/robot_get_device_list.h>
#include <webots_ros/motor_set_control_pid.h>
#include <webots_ros/set_bool.h>
// linux
#include <signal.h>
#include <iostream>

#define TIME_STEP 32
#define POSITION   2
#define VELOCITY   2

// Time Step Client
ros::ServiceClient    ts_client;
webots_ros::set_int   ts_server;
// Keyboard Enable Client
ros::ServiceClient    kbe_client;
webots_ros::set_int   kbe_server;
// Left/Right Motor Position/Velocity Client
static float lpos, rpos;
ros::ServiceClient    lmp_client, rmp_client;
webots_ros::set_float lmp_server, rmp_server;
static float lvel, rvel;
ros::ServiceClient    lmv_client, rmv_client;
webots_ros::set_float lmv_server, rmv_server;
// PID Set Client
ros::ServiceClient    lpid_client, rpid_client;
webots_ros::set_float lpid_server, rpid_server;
// Lidar(Cloud) Enable Client
ros::ServiceClient    lde_client, lce_client;
webots_ros::set_int   lde_server;
webots_ros::set_bool  lce_server;

// user interrupt handle: ctrl-c
void quit(int sig)
{
    kbe_server.request.value = 0;
    ts_server.request.value = 0;
    kbe_client.call(kbe_server);
    ts_client.call(ts_server);
    ROS_INFO("Stopped '/RoombaController' node.");
    ros::shutdown();
    exit(0);
}

// keyboard control directions
void keyboard_callback(const webots_ros::Int32Stamped::ConstPtr &val) 
{
    int send_flag = 0;
    ROS_INFO("key: %d", val->data);
    switch (val->data) {
    case 65: // A
        lpos -= POSITION; rpos += POSITION; send_flag = 1; 
        break;
    case 68: // D
        lpos += POSITION; rpos -= POSITION; send_flag = 1; 
        break;
    case 87: // W
        lpos += POSITION; rpos += POSITION; send_flag = 1; 
        break;
    case 83: // S
        lpos -= POSITION; rpos -= POSITION; send_flag = 1;
        break;
    case 315: // ↑
        lvel += VELOCITY; rvel += VELOCITY; send_flag = 1;
        break;
    case 317: // ↓
        lvel -= VELOCITY; rvel -= VELOCITY; send_flag = 1;
        break;
    }
    if(lvel < 0 || rvel < 0) {
        lvel = rvel = 0;
    } else if(lvel > 16.129 || rvel > 16.129) {
        lvel = rvel = 16.129;
    }
    ROS_INFO("vel: %f", lvel);
    lmp_server.request.value = lpos;
    rmp_server.request.value = rpos;
    lmv_server.request.value = lvel;
    rmv_server.request.value = rvel;
    if (send_flag) {
        if (!lmp_client.call(lmp_server) || !rmp_client.call(rmp_server) || 
            !lmp_server.response.success || !rmp_server.response.success)
            ROS_ERROR("Failed to set position.");
        if (!lmv_client.call(lmv_server) || !rmv_client.call(rmv_server) || 
            !lmv_server.response.success || !rmv_server.response.success)
            ROS_ERROR("Failed to set velocity.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RoombaController");
    ros::NodeHandle hd;
    ros::Subscriber sub_kb;
    ros::Publisher  pub_time;
    // 重新处理ctrl-c中断信号
    signal(SIGINT, quit);
    setlocale(LC_ALL, "");

    // wait for 'ros controller'
    ros::service::waitForService("/Roomba/robot/time_step");
    ros::spinOnce();

// register clients
    // 订阅time_step和webots保持同步
    ts_client  = hd.serviceClient<webots_ros::set_int>("/Roomba/robot/time_step");
    ts_server.request.value  = TIME_STEP;
    kbe_client = hd.serviceClient<webots_ros::set_int>("/Roomba/keyboard/enable");
    kbe_server.request.value = TIME_STEP;
    lmp_client = hd.serviceClient<webots_ros::set_float>("/Roomba/left_wheel_motor/set_position");
    rmp_client = hd.serviceClient<webots_ros::set_float>("/Roomba/right_wheel_motor/set_position");
    lmv_client = hd.serviceClient<webots_ros::set_float>("/Roomba/left_wheel_motor/set_velocity");
    rmv_client = hd.serviceClient<webots_ros::set_float>("/Roomba/right_wheel_motor/set_velocity");
    lde_client = hd.serviceClient<webots_ros::set_int>("/Roomba/laser/enable");
    lde_server.request.value = TIME_STEP;
    lce_client = hd.serviceClient<webots_ros::set_bool>("/Roomba/laser/enable_point_cloud");
    lce_server.request.value = TIME_STEP;
    // publish /clock topic
    pub_time = hd.advertise<rosgraph_msgs::Clock>("/clock", 10);

// enable keyboard
    if (kbe_client.call(kbe_server) && kbe_server.response.success) {
        sub_kb = hd.subscribe("/Roomba/keyboard/key", 1, keyboard_callback);
        while (sub_kb.getNumPublishers() == 0);
        ROS_INFO("Keyboard Enabled. Back to Webots surface and point it, then you can control Roomba");
        ROS_INFO("  w  ");
        ROS_INFO("a s d");
    } else {
        ROS_ERROR("Failed to enable keyboard.");
        goto end_exit;
    }
// enable lidar and lidar cloud
    if(!lde_client.call(lde_server) || !lde_server.response.success || 
       !lce_client.call(lce_server) || !lce_server.response.success) {
        ROS_ERROR("Failed to enable lidar.");
        goto end_exit;
    }


/* main loop */
    ros::Rate(1);
    while(ros::ok()) {
        if (!ts_client.call(ts_server) || !ts_server.response.success) {
            ROS_ERROR("Failed to call service /time_step for next step.");
            break;
        }
        rosgraph_msgs::Clock clk;
        clk.clock = ros::Time::now();
        pub_time.publish(clk); // pub rosgraph_msgs::Clock
        ros::spinOnce();
    }

end_exit:
    ts_server.request.value = 0;
    ts_client.call(ts_server);
    ros::shutdown();
    return 0;
}