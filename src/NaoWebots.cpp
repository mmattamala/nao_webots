/**
 * @file BHInterface.cpp
 * This file implements an UDP communication interface between a NAO and a external computer running ROS
 *
 * This file uses part of the BHuman Code Release 2016 <a href="https://github.com/bhuman/BHumanCodeRelease"</a>)
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */

#ifndef NAOWEBOTS_CPP
#define NAOWEBOTS_CPP

#include "NaoWebots.h"
#include <iostream>

namespace nao_webots{

NaoWebots::NaoWebots() : node_handle_("~")
{
    // Read communication parameters from parameter server
    node_handle_.param("rate_sensors", rate_sensors_, 100);
    node_handle_.param("rate_camera", rate_camera_, 30);

    node_handle_.param("motion_hand_wave", wb_file_motion_hand_wave_, std::string("motions/HandWave.motion"));
    node_handle_.param("motion_forward", wb_file_motion_forward_, std::string("motions/Forwards50.motion"));
    node_handle_.param("motion_backwards", wb_file_motion_backward_, std::string("motions/Backwards.motion"));
    node_handle_.param("motion_side_step_left", wb_file_motion_side_step_left_, std::string("motions/SideStepLeft.motion"));
    node_handle_.param("motion_side_step_right", wb_file_motion_side_step_right_, std::string("motions/SideStepRight.motion"));
    node_handle_.param("motion_turn_left", wb_file_motion_turn_left_, std::string("motions/TurnLeft60.motion"));
    node_handle_.param("motion_turn_right", wb_file_motion_turn_right_, std::string("motions/TurnRight60.motion"));

    // Callbacks
    joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(joyCallback, this, _1));

    // Create communication interfaces
    // IMU
    imu_publisher_ = node_handle_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
    // FSR
    fsr_publisher_ = node_handle_.advertise<geometry_msgs::PointStamped>("fsr", 1);
    // Joints state
    joint_state_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 1);

    // Image topic
    image_transport::ImageTransport it(node_handle_);
    cam_publisher_  = it.advertiseCamera("camera/image_raw", 1);

    // Initialize robot
    time_step_ = getBasicTimeStep();

    // init devices
    initWebotsDevices();

    // load motion files
    loadMotionFiles();
}

NaoWebots::~NaoWebots()
{
    delete wb_motion_hand_wave_;
    delete wb_motion_forward_;
    delete wb_motion_backward_;
    delete wb_motion_side_step_left_;
    delete wb_motion_side_step_right_;
    delete wb_motion_turn_left_;
    delete wb_motion_turn_right_;
}

void NaoWebots::simulationStep()
{
    if (step(time_step_) == -1)
        return;
}

void NaoWebots::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    ROS_INFO("JOY CALLBACK");
}

void NaoWebots::initWebotsDevices()
{
    wb_camera_top_              = boost::shared_ptr<webots::Camera>(getCamera("CameraTop"));
    wb_camera_bottom_           = boost::shared_ptr<webots::Camera>(getCamera("CameraBottom"));
    wb_accelerometer_           = boost::shared_ptr<webots::Accelerometer>(getAccelerometer("accelerometer"));
    wb_gyro_                    = boost::shared_ptr<webots::Gyro>(getGyro("gps"));
    wb_gps_                     = boost::shared_ptr<webots::GPS>(getGPS("gyro"));
    wb_inertial_unit_           = boost::shared_ptr<webots::InertialUnit>(getInertialUnit("inertial unit"));
    wb_sonar_left_              = boost::shared_ptr<webots::DistanceSensor>(getDistanceSensor("Sonar/Left"));
    wb_sonar_right_             = boost::shared_ptr<webots::DistanceSensor>(getDistanceSensor("Sonar/Right"));
    wb_fsr_left_                = boost::shared_ptr<webots::TouchSensor>(getTouchSensor("LFsr"));
    wb_fsr_right_               = boost::shared_ptr<webots::TouchSensor>(getTouchSensor("RFsr"));
    wb_lfoot_bumper_right_      = boost::shared_ptr<webots::TouchSensor>(getTouchSensor("LFoot/Bumper/Left"));
    wb_foot_bumper_left_        = boost::shared_ptr<webots::TouchSensor>(getTouchSensor("LFoot/Bumper/Right"));
    wb_rfoot_bumper_right_      = boost::shared_ptr<webots::TouchSensor>(getTouchSensor("RFoot/Bumper/Left"));
    wb_rfoot_bumper_left_       = boost::shared_ptr<webots::TouchSensor>(getTouchSensor("RFoot/Bumper/Right"));
    wb_head_yaw_sensor_         = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("HeadYawS"));
    wb_head_pitch_sensor_       = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("HeadPitchS"));
    wb_r_shoulder_pitch_sensor_ = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("RShoulderPitchS"));
    wb_r_shoulder_roll_sensor_  = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("RShoulderRollS"));
    wb_r_elbow_yaw_sensor_      = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("RElbowYawS"));
    wb_r_elbow_roll_sensor_     = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("RElbowRollS"));
    wb_l_shoulder_pitch_sensor_ = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("LShoulderPitchS"));
    wb_l_shoulder_roll_sensor_  = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("LShoulderRollS"));
    wb_l_elbow_yaw_sensor_      = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("LElbowYawS"));
    wb_l_elbow_roll_sensor_     = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("LElbowRollS"));
    wb_r_hip_yaw_pitch_sensor_  = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("RHipYawPitchS"));
    wb_r_hip_roll_sensor_       = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("RHipRollS"));
    wb_r_hip_pitch_sensor_      = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("RHipPitchS"));
    wb_r_knee_pitch_sensor_     = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("RKneePitchS"));
    wb_r_ankle_pitch_sensor_    = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("RAnklePitchS"));
    wb_r_ankle_roll_sensor_     = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("RAnkleRollS"));
    wb_l_hip_yaw_pitch_sensor_  = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("LHipYawPitchS"));
    wb_l_hip_roll_sensor_       = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("LHipRollS"));
    wb_l_hip_pitch_sensor_      = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("LHipPitchS"));
    wb_l_knee_pitch_sensor_     = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("LKneePitchS"));
    wb_l_ankle_pitch_sensor_    = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("LAnklePitchS"));
    wb_l_ankle_roll_sensor_     = boost::shared_ptr<webots::PositionSensor>(getPositionSensor("LAnkleRollS"));
    wb_keyboard_                = boost::shared_ptr<webots::Keyboard>(getKeyboard());
}

void NaoWebots::loadMotionFiles()
{
    wb_motion_hand_wave_       = new webots::Motion(wb_file_motion_hand_wave_);
    wb_motion_forward_         = new webots::Motion(wb_file_motion_forward_);
    wb_motion_backward_        = new webots::Motion(wb_file_motion_backward_);
    wb_motion_side_step_left_  = new webots::Motion(wb_file_motion_side_step_left_);
    wb_motion_side_step_right_ = new webots::Motion(wb_motion_side_step_right_);
    wb_motion_turn_left_       = new webots::Motion(wb_motion_turn_left_);
    wb_motion_turn_right_      = new webots::Motion(wb_motion_turn_right_);
}

} //nao_webots

#endif // NAOWEBOTS_CPP
