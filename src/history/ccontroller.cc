/*
 * 梦开始的地方
 */
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Lidar.hpp>
#include <iostream>
#include <fstream>

int main() 
{
    webots::Robot *robot = new webots::Robot();
    int timeStep = (int)robot->getBasicTimeStep();
    std::ofstream log_op;
    log_op.open("log"); 
    log_op << "timeStep: " << timeStep << '\n';
    webots::Motor *lmotor = robot->getMotor("left wheel motor");
    webots::Motor *rmotor = robot->getMotor("right wheel motor");
    webots::Lidar *lidar = robot->getLidar("lidar");
    
    lidar->enable(1);
    lidar->enablePointCloud();
 
    lmotor->setPosition(115.0);
    rmotor->setPosition(115.0);


    while (robot->step(timeStep) != -1) {
      // Read the sensors:
      // Enter here functions to read sensor data, like:
      //  double val = ds->getValue();

      // Process sensor data here.

      // Enter here functions to send actuator commands, like:
      //  motor->setPosition(10.0);
    };

    // Enter here exit cleanup code.

    log_op.close();
    delete robot;
    return 0;
}
