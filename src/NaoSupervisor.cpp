/**
 * @file BHInterface.cpp
 * This file implements an UDP communication interface between a NAO and a external computer running ROS
 *
 * This file uses part of the BHuman Code Release 2016 <a href="https://github.com/bhuman/BHumanCodeRelease"</a>)
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */

#ifndef NAOSUPERVISOR_CPP
#define NAOSUPERVISOR_CPP

#include "NaoSupervisor.h"
#include <iostream>

namespace nao_webots{

NaoSupervisor::NaoSupervisor() :
    node_handle_("~"),
    webots::Supervisor()
{
    // Read communication parameters from parameter server
    node_handle_.param("rate", rate_, 100);

    // Initialize robot
    time_step_ = getBasicTimeStep();
    ROS_WARN("TIME STEP: %d", time_step_);

    wb_robot_node_ = getFromDef("nao");


    // prepare timers for publication
    ros::Timer timer_ground_truth  = node_handle_.createTimer(ros::Duration(1.0/rate_), &NaoSupervisor::callbackGroundTruth, this);

    ros::spin();
}

NaoSupervisor::~NaoSupervisor()
{
}

bool NaoSupervisor::simulationStep()
{
    if (step(time_step_) == -1)
        return false;
}


void NaoSupervisor::callbackGroundTruth(const ros::TimerEvent &event)
{
    wb_robot_node_ = getFromDef("nao");

    ROS_WARN("callback ground truth");
    simulationStep();

    const double* p = wb_robot_node_->getPosition();
    ROS_WARN("got position");
    const double *R = wb_robot_node_->getOrientation();
    ROS_WARN("got orientation");

    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(webotsPositionToROSPosition(p));
    ROS_WARN("set position");
    transform.setRotation(webotsRotationToROSRotation(R));
    ROS_WARN("set orientation");
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "gt_torso"));
    ROS_WARN("sent transform");
}

tf::Vector3 NaoSupervisor::webotsPositionToROSPosition(const double* p)
{
    // The webots frame defines the plane in the X-Z axis, whereas the Y is the normal to the plane.
    // In order to get the position in ROS coordinates, we must set:
    //  |px_ros|   |  px_wb |
    //  |py_ros| = | -pz_wb |
    //  |pz_ros|   |  py_wb |

    return tf::Vector3(p[0], -p[2], p[1]);
}

tf::Quaternion NaoSupervisor::webotsRotationToROSRotation(const double* R)
{
    // The webots frame defines the plane in the X-Z axis, whereas the Y is the normal to the plane.
    // Webots coordinates define:
    //  | R[0] R[1] R[2] |
    //  | R[3] R[4] R[5] |
    //  | R[6] R[7] R[8] |

    // To transform this into ROS coordinates, we must swap and change signs of some columns:
    //  | R[0] -R[2] R[1] |
    //  | R[3] -R[5] R[4] |
    //  | R[6] -R[8] R[7] |

    tf::Matrix3x3 Rot;
    Rot.setValue(R[0], -R[2], R[1], R[3], -R[5], R[4], R[6], -R[8], R[7]);
    double roll, pitch, yaw;
    Rot.getRPY(roll, pitch, yaw);

    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}





} //nao_webots

#endif // NAOWEBOTS_CPP
