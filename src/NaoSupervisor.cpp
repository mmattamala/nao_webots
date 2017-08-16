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
    node_handle_.param("robot_name", robot_name_, std::string("NAO_ROBOT"));
    node_handle_.param("use_torso_gt_tf", use_torso_gt_tf_, false);

    // Initialize robot
    time_step_ = getBasicTimeStep();
    ROS_WARN("TIME STEP: %d", time_step_);

    wb_robot_node_ = getFromDef(robot_name_);

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
    //wb_robot_node_ = getFromDef(robot_name_);

    simulationStep();

    const double* p = wb_robot_node_->getPosition();
    const double *R = wb_robot_node_->getOrientation();

    static tf::TransformBroadcaster br;
    tf::Transform Tgt;
    Tgt.setOrigin(webotsPositionToROSPosition(p));
    Tgt.setRotation(webotsRotationToROSRotation(R));
    br.sendTransform(tf::StampedTransform(Tgt, ros::Time::now(), "world", "gt_torso"));

    if(use_torso_gt_tf_)
    {
        tf::Transform Tgt_t;
        Tgt_t.setOrigin(tf::Vector3(0,0,0));
        Tgt_t.setRotation(tf::Quaternion(0,0,0,1.0));
        br.sendTransform(tf::StampedTransform(Tgt_t, ros::Time::now(), "gt_torso", "base_link"));
    }
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
    Rot.setValue( R[0],  R[1],  R[2],
                 -R[6], -R[7], -R[8],
                  R[3],  R[4],  R[5]);
    double roll, pitch, yaw;
    Rot.getRPY(roll, pitch, yaw);

    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}





} //nao_webots

#endif // NAOWEBOTS_CPP
