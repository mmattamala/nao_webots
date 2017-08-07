/**
 * @file NaoSupervisor.h
 * This file implements a communication interface between ROS and Webots
 *
 * It is based on the NAO Demo C file available in Webots
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */

 #ifndef NAOWEBOTS_H
 #define NAOWEBOTS_H

// Webots stuff
#include <webots/utils/Motion.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/PositionSensor.hpp>

#include <webots/Supervisor.hpp>

// ROS Dependencies
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

// Boost
//#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

// Other C++ dependencies
#include <unistd.h>

namespace nao_webots {

class NaoSupervisor :
        public webots::Supervisor
{
public:
    // Default Constructor
    NaoSupervisor();

    // Default destructor
    ~NaoSupervisor();

private:
    // callbacks
    void callbackGroundTruth(const ros::TimerEvent& event);

    // run simulation step
    bool simulationStep();

    // Utils
    tf::Vector3 webotsPositionToROSPosition(const double* p);
    tf::Quaternion webotsRotationToROSRotation(const double* R);

private:
    int time_step_;
    std::string robot_name_;

    // Node handle
    ros::NodeHandle node_handle_;

    // parameters
    int rate_;
    bool use_torso_gt_tf_;

    // Robot node
    webots::Node* wb_robot_node_;
    webots::Field* wb_field;
};


} //nao_webots
#endif // NAOWEBOTS_H
