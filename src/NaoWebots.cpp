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

NaoWebots::NaoWebots() : node_handle_("~"), Robot()
{
    // Read communication parameters from parameter server
    node_handle_.param("rate_sensors", rate_sensors_, 100);
    node_handle_.param("rate_camera", rate_camera_, 30);

    node_handle_.param("motion_hand_wave", wb_file_motion_hand_wave_, std::string("../../motions/HandWave.motion"));
    node_handle_.param("motion_forward", wb_file_motion_forward_, std::string("../../motions/Forwards50.motion"));
    node_handle_.param("motion_backwards", wb_file_motion_backward_, std::string("../../motions/Backwards.motion"));
    node_handle_.param("motion_side_step_left", wb_file_motion_side_step_left_, std::string("../../motions/SideStepLeft.motion"));
    node_handle_.param("motion_side_step_right", wb_file_motion_side_step_right_, std::string("../../motions/SideStepRight.motion"));
    node_handle_.param("motion_turn_left", wb_file_motion_turn_left_, std::string("../../motions/TurnLeft60.motion"));
    node_handle_.param("motion_turn_right", wb_file_motion_turn_right_, std::string("../../motions/TurnRight60.motion"));

    // Callbacks
    joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, &NaoWebots::callbackJoy, this);

    // Create communication interfaces
    // IMU
    imu_publisher_ = node_handle_.advertise<sensor_msgs::Imu>("/imu/data_raw", 1);
    // FSR
    fsr_publisher_ = node_handle_.advertise<geometry_msgs::PointStamped>("/fsr", 1);
    // Joints state
    joint_state_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 1);

    // Image topic
    image_transport::ImageTransport it(node_handle_);
    cam_publisher_  = it.advertiseCamera("camera/image_raw", 1);

    // Initialize robot
    time_step_ = getBasicTimeStep();
    ROS_INFO("TIME STEP: %d", time_step_);

    // init devices
    initWebotsDevices();

    // load motion files
    loadMotionFiles();

    // prepare timers for publication
    ros::Timer timer_sensors = node_handle_.createTimer(ros::Duration(1.0/rate_sensors_), &NaoWebots::callbackSensors, this);
    ros::Timer timer_camera  = node_handle_.createTimer(ros::Duration(1.0/rate_camera_), &NaoWebots::callbackCamera, this);

    ros::spin();
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

void NaoWebots::callbackCamera(const ros::TimerEvent& event)
{
    simulationStep();

    ros::Time time = ros::Time::now();

    int image_width = wb_camera_top_->getWidth();
    int image_height = wb_camera_top_->getHeight();

    // read rgb pixel values from the camera
    const unsigned char *image = wb_camera_top_->getImage();


    sensor_msgs::Image ros_image;
    sensor_msgs::CameraInfo ros_cam_info;

    // fill image
    ros_image.encoding = "rgb8";
    ros_image.height   = image_height;
    ros_image.width    = image_width;
    ros_image.step     = 3 * image_width;

    size_t st0 = (ros_image.step * ros_image.height);
    ros_image.data.resize(st0);
    ros_image.is_bigendian = 0;

    for (int y = 0; y < image_height; y++)
    {
        for (int x = 0; x < image_width; x++)
        {
            ros_image.data[3*(y*image_width + x)    ] = wb_camera_top_->imageGetRed(image, image_width, x, y);
            ros_image.data[3*(y*image_width + x) + 1] = wb_camera_top_->imageGetGreen(image, image_width, x, y);
            ros_image.data[3*(y*image_width + x) + 2] = wb_camera_top_->imageGetBlue(image, image_width, x, y);
        }
    }

    // fill camera info
    ros_cam_info.height = image_height;
    ros_cam_info.width = image_width;
    ros_cam_info.header.stamp = time;
    ros_cam_info.header.frame_id = "/CameraTop_optical_frame";

    // publish image
    cam_publisher_.publish(ros_image, ros_cam_info);
}

void NaoWebots::callbackSensors(const ros::TimerEvent& event)
{
    simulationStep();

    ros::Time time = ros::Time::now();

    //ROS_INFO("SENSORS CALLBACK");
    sensor_msgs::JointState ros_joints;

    ros_joints.name.resize(26);
    ros_joints.position.resize(26);

    // Head angles
    ros_joints.name[0]      = "HeadYaw";
    ros_joints.position[0]  = wb_head_yaw_sensor_->getValue();

    ros_joints.name[1]      = "HeadPitch";
    ros_joints.position[1]  = wb_head_pitch_sensor_->getValue();

    // Left leg
    ros_joints.name[2]      = "LHipYawPitch";
    ros_joints.position[2]  = wb_l_hip_yaw_pitch_sensor_->getValue();

    ros_joints.name[3]      = "LHipRoll";
    ros_joints.position[3]  = wb_l_hip_roll_sensor_->getValue();

    ros_joints.name[4]      = "LHipPitch";
    ros_joints.position[4]  = wb_l_hip_pitch_sensor_->getValue();

    ros_joints.name[5]      = "LKneePitch";
    ros_joints.position[5]  = wb_l_knee_pitch_sensor_->getType();

    ros_joints.name[6]      = "LAnklePitch";
    ros_joints.position[6]  = wb_l_ankle_pitch_sensor_->getValue();

    ros_joints.name[7]      = "LAnkleRoll";
    ros_joints.position[7]  = wb_l_ankle_roll_sensor_->getValue();

    // Right Leg
    ros_joints.name[8]      = "RHipYawPitch";
    ros_joints.position[8]  = wb_r_hip_yaw_pitch_sensor_->getValue();

    ros_joints.name[9]      = "RHipRoll";
    ros_joints.position[9]  = wb_r_hip_roll_sensor_->getValue();

    ros_joints.name[10]     = "RHipPitch";
    ros_joints.position[10] = wb_r_hip_pitch_sensor_->getValue();

    ros_joints.name[11]     = "RKneePitch";
    ros_joints.position[11] = wb_r_knee_pitch_sensor_->getValue();

    ros_joints.name[12]     = "RAnklePitch";
    ros_joints.position[12] = wb_r_ankle_pitch_sensor_->getValue();

    ros_joints.name[13]     = "RAnkleRoll";
    ros_joints.position[13] = wb_r_ankle_roll_sensor_->getValue();

    // Left arm
    ros_joints.name[14]     = "LShoulderPitch";
    ros_joints.position[14] = wb_l_shoulder_pitch_sensor_->getValue();

    ros_joints.name[15]     = "LShoulderRoll";
    ros_joints.position[15] = wb_l_shoulder_roll_sensor_->getValue();

    ros_joints.name[16]     = "LElbowYaw";
    ros_joints.position[16] = wb_l_elbow_yaw_sensor_->getValue();

    ros_joints.name[17]     = "LElbowRoll";
    ros_joints.position[17] = wb_l_elbow_roll_sensor_->getValue();

    ros_joints.name[18]     = "LWristYaw";
    ros_joints.position[18] = 0.0;

    ros_joints.name[19]     = "LHand";
    ros_joints.position[19] = 0.0;

    // Right arm
    ros_joints.name[20]     = "RShoulderPitch";
    ros_joints.position[20] = wb_r_shoulder_pitch_sensor_->getValue();

    ros_joints.name[21]     = "RShoulderRoll";
    ros_joints.position[21] = wb_r_shoulder_roll_sensor_->getValue();

    ros_joints.name[22]     = "RElbowYaw";
    ros_joints.position[22] = wb_r_elbow_yaw_sensor_->getValue();

    ros_joints.name[23]     = "RElbowRoll";
    ros_joints.position[23] = wb_r_elbow_roll_sensor_->getValue();

    ros_joints.name[24]     = "RWristYaw";
    ros_joints.position[24] = 0.0;

    ros_joints.name[25]     = "RHand";
    ros_joints.position[25] = 0.0;

    // fill header
    ros_joints.header.stamp = time;
    joint_state_publisher_.publish(ros_joints);
}

void NaoWebots::callbackJoy(const sensor_msgs::Joy::ConstPtr& joy)
{
    simulationStep();
}

void NaoWebots::initWebotsDevices()
{
    wb_camera_top_              = getCamera("CameraTop");
    wb_camera_bottom_           = getCamera("CameraBottom");
    wb_accelerometer_           = getAccelerometer("accelerometer");
    wb_gyro_                    = getGyro("gyro");
    wb_gps_                     = getGPS("gps");
    wb_inertial_unit_           = getInertialUnit("inertial unit");
    wb_sonar_left_              = getDistanceSensor("Sonar/Left");
    wb_sonar_right_             = getDistanceSensor("Sonar/Right");
    wb_fsr_left_                = getTouchSensor("LFsr");
    wb_fsr_right_               = getTouchSensor("RFsr");
    wb_lfoot_bumper_right_      = getTouchSensor("LFoot/Bumper/Left");
    wb_foot_bumper_left_        = getTouchSensor("LFoot/Bumper/Right");
    wb_rfoot_bumper_right_      = getTouchSensor("RFoot/Bumper/Left");
    wb_rfoot_bumper_left_       = getTouchSensor("RFoot/Bumper/Right");
    wb_head_yaw_sensor_         = getPositionSensor("HeadYawS");
    wb_head_pitch_sensor_       = getPositionSensor("HeadPitchS");
    wb_r_shoulder_pitch_sensor_ = getPositionSensor("RShoulderPitchS");
    wb_r_shoulder_roll_sensor_  = getPositionSensor("RShoulderRollS");
    wb_r_elbow_yaw_sensor_      = getPositionSensor("RElbowYawS");
    wb_r_elbow_roll_sensor_     = getPositionSensor("RElbowRollS");
    wb_l_shoulder_pitch_sensor_ = getPositionSensor("LShoulderPitchS");
    wb_l_shoulder_roll_sensor_  = getPositionSensor("LShoulderRollS");
    wb_l_elbow_yaw_sensor_      = getPositionSensor("LElbowYawS");
    wb_l_elbow_roll_sensor_     = getPositionSensor("LElbowRollS");
    wb_r_hip_yaw_pitch_sensor_  = getPositionSensor("RHipYawPitchS");
    wb_r_hip_roll_sensor_       = getPositionSensor("RHipRollS");
    wb_r_hip_pitch_sensor_      = getPositionSensor("RHipPitchS");
    wb_r_knee_pitch_sensor_     = getPositionSensor("RKneePitchS");
    wb_r_ankle_pitch_sensor_    = getPositionSensor("RAnklePitchS");
    wb_r_ankle_roll_sensor_     = getPositionSensor("RAnkleRollS");
    wb_l_hip_yaw_pitch_sensor_  = getPositionSensor("LHipYawPitchS");
    wb_l_hip_roll_sensor_       = getPositionSensor("LHipRollS");
    wb_l_hip_pitch_sensor_      = getPositionSensor("LHipPitchS");
    wb_l_knee_pitch_sensor_     = getPositionSensor("LKneePitchS");
    wb_l_ankle_pitch_sensor_    = getPositionSensor("LAnklePitchS");
    wb_l_ankle_roll_sensor_     = getPositionSensor("LAnkleRollS");
    wb_keyboard_                = getKeyboard();

    // enable devices
    wb_camera_top_->enable(time_step_);
    wb_camera_bottom_->enable(time_step_);
    wb_accelerometer_->enable(time_step_);
    wb_gyro_->enable(time_step_);
    wb_gps_->enable(time_step_);
    wb_inertial_unit_->enable(time_step_);
    wb_sonar_left_->enable(time_step_);
    wb_sonar_right_->enable(time_step_);
    wb_fsr_left_->enable(time_step_);
    wb_fsr_right_->enable(time_step_);
    wb_lfoot_bumper_right_->enable(time_step_);
    wb_foot_bumper_left_->enable(time_step_);
    wb_rfoot_bumper_right_->enable(time_step_);
    wb_rfoot_bumper_left_->enable(time_step_);
    wb_head_yaw_sensor_->enable(time_step_);
    wb_head_pitch_sensor_->enable(time_step_);
    wb_r_shoulder_pitch_sensor_->enable(time_step_);
    wb_r_shoulder_roll_sensor_->enable(time_step_);
    wb_r_elbow_yaw_sensor_->enable(time_step_);
    wb_r_elbow_roll_sensor_->enable(time_step_);
    wb_l_shoulder_pitch_sensor_->enable(time_step_);
    wb_l_shoulder_roll_sensor_->enable(time_step_);
    wb_l_elbow_yaw_sensor_->enable(time_step_);
    wb_l_elbow_roll_sensor_->enable(time_step_);
    wb_r_hip_yaw_pitch_sensor_->enable(time_step_);
    wb_r_hip_roll_sensor_->enable(time_step_);
    wb_r_hip_pitch_sensor_->enable(time_step_);
    wb_r_knee_pitch_sensor_->enable(time_step_);
    wb_r_ankle_pitch_sensor_->enable(time_step_);
    wb_r_ankle_roll_sensor_->enable(time_step_);
    wb_l_hip_yaw_pitch_sensor_->enable(time_step_);
    wb_l_hip_roll_sensor_->enable(time_step_);
    wb_l_hip_pitch_sensor_->enable(time_step_);
    wb_l_knee_pitch_sensor_->enable(time_step_);
    wb_l_ankle_pitch_sensor_->enable(time_step_);
    wb_l_ankle_roll_sensor_->enable(time_step_);
    wb_keyboard_->enable(time_step_);
}

void NaoWebots::loadMotionFiles()
{
    wb_motion_hand_wave_       = new webots::Motion(wb_file_motion_hand_wave_);
    wb_motion_forward_         = new webots::Motion(wb_file_motion_forward_);
    wb_motion_backward_        = new webots::Motion(wb_file_motion_backward_);
    wb_motion_side_step_left_  = new webots::Motion(wb_file_motion_side_step_left_);
    wb_motion_side_step_right_ = new webots::Motion(wb_file_motion_side_step_right_);
    wb_motion_turn_left_       = new webots::Motion(wb_file_motion_turn_left_);
    wb_motion_turn_right_      = new webots::Motion(wb_file_motion_turn_right_);
}

} //nao_webots

#endif // NAOWEBOTS_CPP
