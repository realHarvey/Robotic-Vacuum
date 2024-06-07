/**
 * class Robot and its implement
 * 按 webots 风格创建
 */
#ifndef ROBOT_HPP_
#define ROBOT_HPP_
// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// webots_ros
#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/set_bool.h>
#define VELOCITY 0.785

class Robot {
public:
    Robot(std::string name, ros::NodeHandle *hd, int time_step, std::string dev[3]);
    virtual ~Robot() = default;
/* dev enable */
    void initMotor();
    void enableLidar();
    void enableKeyboard();
    void enableSelfDriving();
    void enableIMU();
    void enableGyro();
    void enableGPS();
    void publishOdom();
    void checkTimeStep();
    // user interrupt handle: ctrl-c
    void quit();
    static void signalInterrupt(int sig);
private:
    template<typename T>
    void enableService(std::string service_name, ros::ServiceClient client);
    void keyboard_callback(const webots_ros::Int32Stamped::ConstPtr &val);
    void imu_callback(const sensor_msgs::Imu::ConstPtr &val);
    void gyro_callback(const sensor_msgs::Imu::ConstPtr &val);
    void gps_callback(const geometry_msgs::PointStamped::ConstPtr &val);
    void gpsv_callback(const geometry_msgs::PointStamped::ConstPtr &val);
    void selfdriving_callback(const geometry_msgs::Twist::ConstPtr &val);
public:
private:
    // only for simpl-use
    ros::NodeHandle *hd;
    // time step: sync ros and webots
    int time_step;
    ros::ServiceClient ts_client;
    // robot
    std::string nRobot;
    // motor
    std::string nLeftMotor, nRightMotor;
    ros::ServiceClient lmv_client, rmv_client;
    float lvel, rvel, max_vel;
    // lidar
    std::string nLidar;
    ros::ServiceClient lde_client, lce_client;
    // keyboard
    ros::ServiceClient kb_client;
    // odom: imu
    geometry_msgs::Quaternion imu_quat;
    double angle_vel;
    ros::ServiceClient imu_client, gyro_client;
    // odom
    ros::Publisher pub_odom;
    // gps: value, flag...
    double gps_values[4], gpsv_x, gpsv_y;
    int GF;
    ros::ServiceClient gps_client;
};

Robot::Robot(std::string name, ros::NodeHandle *hd, int time_step, std::string dev[3])
{
    webots_ros::set_int ts_server;
    this->hd = hd;
    this->time_step = time_step;
    nRobot = name.insert(0, "/");
    ros::service::waitForService(nRobot + "/robot/time_step");
    ros::spinOnce();
    // 订阅 /time_step 和 webots 保持同步
    ts_client = hd->serviceClient<webots_ros::set_int>(nRobot + "/robot/time_step");
    ts_server.request.value = time_step;
    // device_name
    nLeftMotor  = dev[0].insert(0, "/");
    nRightMotor = dev[1].insert(0, "/");
    nLidar = dev[2].insert(0, "/");
    // initialize l/rvel
    rvel = 0.0f;
    lvel = 0.0f;
    pub_odom = hd->advertise<nav_msgs::Odometry>("odom", 10);
    GF = 1;
}

void Robot::initMotor()
{
    ros::ServiceClient    lmp_client, rmp_client;
    webots_ros::set_float lmp_server, rmp_server;
    webots_ros::set_float lmv_server, rmv_server;
    lmp_client = hd->serviceClient<webots_ros::set_float>(nRobot + nLeftMotor  + "/set_position");
    rmp_client = hd->serviceClient<webots_ros::set_float>(nRobot + nRightMotor + "/set_position");
    lmv_client = hd->serviceClient<webots_ros::set_float>(nRobot + nLeftMotor  + "/set_velocity");
    rmv_client = hd->serviceClient<webots_ros::set_float>(nRobot + nRightMotor + "/set_velocity");
    lmp_server.request.value = INFINITY; // set to Infinity, use Velocity to control
    rmp_server.request.value = INFINITY;
    lmv_server.request.value = 0.0f;
    rmv_server.request.value = 0.0f;
    // reset velocity first, then infinite position
    if (!lmv_client.call(lmv_server) || !rmv_client.call(rmv_server) || 
        !lmv_server.response.success || !rmv_server.response.success)
        ROS_ERROR("Failed to reset velocity.");
    if (!lmp_client.call(lmp_server) || !rmp_client.call(rmp_server) || 
        !lmp_server.response.success || !rmp_server.response.success)
        ROS_ERROR("Failed to infinite position.");
}

template<typename T>
void Robot::enableService(std::string service_name, ros::ServiceClient client)
{
    T server;
    client = hd->serviceClient<T>(nRobot + service_name);
    server.request.value = time_step;
    if(!client.call(server) || !server.response.success) {
        ROS_ERROR("Failed to enable %s", service_name.c_str());
        quit();
    }
}

void Robot::enableLidar()
{
    enableService<webots_ros::set_int>(nLidar + "/enable", lde_client);
    enableService<webots_ros::set_bool>(nLidar + "/enable_point_cloud", lce_client);
}

void Robot::enableKeyboard()
{
    static ros::Subscriber sub_kb;
    enableService<webots_ros::set_int>("/keyboard/enable", kb_client);
    // 订阅按键话题: /key
    sub_kb = hd->subscribe<webots_ros::Int32Stamped>("/Roomba/keyboard/key", 1,
        [&](const webots_ros::Int32Stamped::ConstPtr &val){return Robot::keyboard_callback(val);});
    while(sub_kb.getNumPublishers() == 0);
    ROS_INFO("Keyboard Enabled: Back to Webots surface and point it, then you can control Roomba");
    ROS_INFO("  w  ");
    ROS_INFO("a s d");
}

// keyboard control directions
void Robot::keyboard_callback(const webots_ros::Int32Stamped::ConstPtr &val) 
{
    int SF = 0; // send flag: 1 -> can send
    webots_ros::set_float lmv_server, rmv_server;
    //ROS_INFO("key: %d", val->data);
    switch (val->data) {
    case 87: // W: 加速
        lvel += VELOCITY; rvel += VELOCITY; SF = 1; break;
    case 83: // S: 减速
        lvel -= VELOCITY; rvel -= VELOCITY; SF = 1; break;
    case 65: // A: 左转
        rvel += VELOCITY; lvel -= VELOCITY; SF = 1; break;
    case 68: // D: 右转
        lvel += VELOCITY; rvel -= VELOCITY; SF = 1; break;
    case 32: // Space: 方向回正
        lvel = lvel > rvel ? lvel : rvel;
        rvel = lvel; SF = 1; break;
    case 66: // B: brake刹车
        lvel = rvel = 0; SF = 1; break;
    }
    if(lvel > 16) lvel = 16; else if(lvel < -16) lvel = -16;
    if(rvel > 16) rvel = 16; else if(rvel < -16) rvel = -16;
    ROS_INFO("lvel: %f\nrvel: %f", lvel, rvel);
    lmv_server.request.value = lvel;
    rmv_server.request.value = rvel;
    if(SF) {
        if (!lmv_client.call(lmv_server) || !rmv_client.call(rmv_server) || 
            !lmv_server.response.success || !rmv_server.response.success)
            ROS_ERROR("Failed to set velocity.");
    }
}

// 建图时请不要开启自动驾驶
void Robot::enableSelfDriving()
{
    static ros::Subscriber sub_sd;
    sub_sd = hd->subscribe("/cmd_vel", 1, &Robot::selfdriving_callback, this);
    while(sub_sd.getNumPublishers() == 0);
    ROS_INFO("Self Driving Enabled.");
}

void Robot::selfdriving_callback(const geometry_msgs::Twist::ConstPtr &val)
{
    float v, w, lvel, rvel, L = 0.2718;
    webots_ros::set_float lmv_server, rmv_server;
    v = val->linear.x;
    w = -val->angular.z; // webots的模型问题导致此处需要反向转动
    //ROS_INFO("v = %f, w = %f", v, w);
    // 分别计算两轮的速度
    lvel = 20.0 * (v + w*L/2);
    rvel = 20.0 * (v - w*L/2);
    //ROS_INFO("l = %f, r = %f", lvel, rvel);
    lmv_server.request.value = lvel;
    rmv_server.request.value = rvel;
    if (!lmv_client.call(lmv_server) || !rmv_client.call(rmv_server) || 
        !lmv_server.response.success || !rmv_server.response.success)
        ROS_ERROR("Failed in Self-Driving.");
}

void Robot::enableIMU()
{
    static ros::Subscriber sub_imu;
    enableService<webots_ros::set_int>("/imu/enable", imu_client);
    sub_imu = hd->subscribe<sensor_msgs::Imu>("/Roomba/imu/quaternion", 1, &Robot::imu_callback, this);
    while(sub_imu.getNumPublishers() == 0);
    ROS_INFO("IMU Enabled.");
}

void Robot::imu_callback(const sensor_msgs::Imu::ConstPtr &val)
{
    imu_quat = val->orientation;
}

void Robot::enableGyro()
{
    static ros::Subscriber sub_gyro;
    enableService<webots_ros::set_int>("/gyro/enable", imu_client);
    sub_gyro = hd->subscribe<sensor_msgs::Imu>("/Roomba/gyro/values", 1, &Robot::gyro_callback, this);
    while(sub_gyro.getNumPublishers() == 0);
    ROS_INFO("Gyro Enabled.");
}

void Robot::gyro_callback(const sensor_msgs::Imu::ConstPtr &val)
{
    angle_vel = val->angular_velocity.z;
}

void Robot::enableGPS()
{
    static ros::Subscriber sub_gps, sub_gpsv;
    enableService<webots_ros::set_int>("/gps/enable", gps_client);
    sub_gps = hd->subscribe("/Roomba/gps/values", 1, &Robot::gps_callback, this);
    while(sub_gps.getNumPublishers() == 0);
    ROS_INFO("GPS Enabled.");
    sub_gpsv = hd->subscribe("/Roomba/gps/speed_vector", 1, &Robot::gpsv_callback, this);
    while(sub_gpsv.getNumPublishers() == 0);
    ROS_INFO("GPSV Enabled.");
}

void Robot::gps_callback(const geometry_msgs::PointStamped::ConstPtr &val)
{
    gps_values[0] = val->point.x;
    gps_values[1] = val->point.y;
    // 初始点位置
    if(GF) {
        gps_values[2] = val->point.x;
        gps_values[3] = val->point.y;
        GF = 0;
    }
    publishOdom();
}

void Robot::gpsv_callback(const geometry_msgs::PointStamped::ConstPtr &val)
{
    gpsv_x = val->point.x;
    gpsv_y = val->point.y;
}

void Robot::publishOdom()
{
    static tf::TransformBroadcaster bc;
    double x = gps_values[0] - gps_values[2];
    double y = gps_values[1] - gps_values[3];
    tf::Transform tf_;
    tf::Quaternion q(imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w);
    nav_msgs::Odometry odom;

/* First, publish the transform over tf: ros 收到广播消息后, 将位置关系插入tf_tree */
    tf_.setOrigin(tf::Vector3(x, y, 0.0));
    // set quaternion of pose about robot
    tf_.setRotation(q);
    // 广播 base_link(robot坐标系) 相对于 odom(里程计坐标系) 的坐标系转换
    bc.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "odom", "base_link"));
    // 广播 lidar、imu -> base_link 的 TF(0,0,0)
    tf_.setIdentity();
    bc.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "base_link", "Roomba/laser"));
    bc.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "base_link", "Roomba/imu"));
    bc.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "base_link", "Roomba/acc"));
    bc.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "base_link", "Roomba/gps"));
    bc.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "base_link", "Roomba/gyro"));
    
/* Next, publish the odometry msg over ros */
    odom.header.stamp    = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";
    // set position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = imu_quat;
    // set velocity
    odom.twist.twist.linear.x = gpsv_x;
    odom.twist.twist.linear.y = gpsv_y;
    odom.twist.twist.angular.z = angle_vel;

    pub_odom.publish(odom);
}

void Robot::checkTimeStep()
{
    webots_ros::set_int ts_server;
    ts_server.request.value = time_step;
    if(!ts_client.call(ts_server) || !ts_server.response.success) {
        ROS_ERROR("Failed to call service /time_step for next step.");
        quit();
    }
}

void Robot::quit()
{
    // value auto init to 0 in constructer
    webots_ros::set_int ts_server;
    ts_client.call(ts_server);
    ROS_INFO("Stopped %s node.", nRobot.c_str());
    ros::shutdown();
    exit(0);
}

void Robot::signalInterrupt(int sig)
{
    ROS_INFO("Stopped this node.");
    ros::shutdown();
    exit(0);
}

#endif
