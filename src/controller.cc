#include <Robot.hpp>
// ros
#include <rosgraph_msgs/Clock.h>
#include <rosgraph_msgs/Log.h>
// linux
#include <signal.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle hd;
    std::string device[3] = {
        "left_wheel_motor",
        "right_wheel_motor",
        "laser"
    };
    Robot rb("Roomba", &hd, 32, device);
    // 重新处理ctrl-c中断信号
    signal(SIGINT, Robot::signalInterrupt);
    setlocale(LC_ALL, "");

    rb.initMotor();
    rb.enableLidar();
    rb.enableKeyboard();
    rb.enableIMU();
    rb.enableGPS();
    rb.enableGyro();
    //rb.enableSelfDriving();

/* main loop */
    ros::Rate(1);
    while(ros::ok()) {
        rb.checkTimeStep();
        ros::spinOnce();
    }
    rb.quit();
}
