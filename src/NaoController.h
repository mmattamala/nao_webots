/**
 * @file NaoWebots.h
 * This file implements a communication interface between ROS and Webots
 *
 * It is based on the NAO Demo C file available in Webots
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */

 #ifndef NAOCONTROLLER_H
 #define NAOCONTROLLER_H

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

// ROS Dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <naoqi_bridge_msgs/FloatArrayStamped.h>

// Boost
//#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

// Other C++ dependencies
#include <unistd.h>

namespace nao_webots {

class NaoController :
        public webots::Robot
{
public:
    // Default Constructor
    NaoController();

    // Default destructor
    ~NaoController();

private:
    // callbacks
    void callbackJoy(const sensor_msgs::Joy::ConstPtr& joy);
    void callbackCamera(const ros::TimerEvent& event);
    void callbackSensors(const ros::TimerEvent& event);
    void callbackKeyboard(const ros::TimerEvent& event);

    // Print currently used parameters
    void printParameters();

    // Initialize webots devices
    void initWebotsDevices();

    // Load motion files
    void loadMotionFiles();
    void startMotion(webots::Motion* motion);

    // Publish ROS data
    void publishCamera(ros::Time &time, webots::Camera *webots_camera, image_transport::CameraPublisher &camera_publisher, const std::string &topic);
    void publishJointState(ros::Time& time);
    void publishIMU(ros::Time& time);
    void publishFSR(ros::Time& time);
    void publishFSR2(ros::Time& time);
    void publishGroundTruth(ros::Time& time);

    // run simulation step
    bool simulationStep();

    // utils
    double clamp(double value, double min, double max);


private:
    int time_step_;

    // Node handle
    ros::NodeHandle node_handle_;

    // ROS publishers and subscribers
    ros::Publisher imu_publisher_;
    ros::Publisher joint_state_publisher_;
    ros::Publisher fsr_l_publisher_;
    ros::Publisher fsr_r_publisher_;
    ros::Publisher fsr_publisher_;
    image_transport::CameraPublisher cam_upper_publisher_;
    image_transport::CameraPublisher cam_lower_publisher_;

    ros::Subscriber joy_subscriber_;

    // parameters
    int rate_sensors_;
    int rate_camera_;

    std::string wb_file_motion_hand_wave_;
    std::string wb_file_motion_forward_;
    std::string wb_file_motion_backward_;
    std::string wb_file_motion_side_step_left_;
    std::string wb_file_motion_side_step_right_;
    std::string wb_file_motion_turn_left_;
    std::string wb_file_motion_turn_right_;

    // Webots Motions
    webots::Motion* wb_motion_current_;

    webots::Motion* wb_motion_hand_wave_;
    webots::Motion* wb_motion_forward_;
    webots::Motion* wb_motion_backward_;
    webots::Motion* wb_motion_side_step_left_;
    webots::Motion* wb_motion_side_step_right_;
    webots::Motion* wb_motion_turn_left_;
    webots::Motion* wb_motion_turn_right_;

    // Webots devices
    // Cameras
    webots::Camera* wb_camera_top_;
    webots::Camera* wb_camera_bottom_;

    // Accelerometer
    webots::Accelerometer* wb_accelerometer_;

    // Gyro
    webots::Gyro* wb_gyro_;

    // GPS (ground truth)
    webots::GPS* wb_gps_;

    // Inertial Unit (Orientation)
    webots::InertialUnit* wb_inertial_unit_;

    // Ultrasound sensors
    webots::DistanceSensor* wb_sonar_left_;
    webots::DistanceSensor* wb_sonar_right_;

    // Foot sensors
    webots::TouchSensor* wb_fsr_left_;
    webots::TouchSensor* wb_fsr_right_;

    // Foot bumpers
    webots::TouchSensor* wb_lfoot_bumper_right_;
    webots::TouchSensor* wb_foot_bumper_left_;
    webots::TouchSensor* wb_rfoot_bumper_right_;
    webots::TouchSensor* wb_rfoot_bumper_left_;

    // Joint encoders
    std::vector<webots::PositionSensor*> wb_joints_;
    webots::PositionSensor* wb_head_yaw_sensor_;
    webots::PositionSensor* wb_head_pitch_sensor_;
    webots::PositionSensor* wb_r_shoulder_pitch_sensor_;
    webots::PositionSensor* wb_r_shoulder_roll_sensor_;
    webots::PositionSensor* wb_r_elbow_yaw_sensor_;
    webots::PositionSensor* wb_r_elbow_roll_sensor_;
    webots::PositionSensor* wb_l_shoulder_pitch_sensor_;
    webots::PositionSensor* wb_l_shoulder_roll_sensor_;
    webots::PositionSensor* wb_l_elbow_yaw_sensor_;
    webots::PositionSensor* wb_l_elbow_roll_sensor_;
    webots::PositionSensor* wb_r_hip_yaw_pitch_sensor_;
    webots::PositionSensor* wb_r_hip_roll_sensor_;
    webots::PositionSensor* wb_r_hip_pitch_sensor_;
    webots::PositionSensor* wb_r_knee_pitch_sensor_;
    webots::PositionSensor* wb_r_ankle_pitch_sensor_;
    webots::PositionSensor* wb_r_ankle_roll_sensor_;
    webots::PositionSensor* wb_l_hip_yaw_pitch_sensor_;
    webots::PositionSensor* wb_l_hip_roll_sensor_;
    webots::PositionSensor* wb_l_hip_pitch_sensor_;
    webots::PositionSensor* wb_l_knee_pitch_sensor_;
    webots::PositionSensor* wb_l_ankle_pitch_sensor_;
    webots::PositionSensor* wb_l_ankle_roll_sensor_;

    // Keyboard
    webots::Keyboard* wb_keyboard_;
};


} //nao_webots
#endif // NAOCONTROLLER_H
