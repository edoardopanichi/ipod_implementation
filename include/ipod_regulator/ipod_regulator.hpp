/* Quadcopter Model Predictive Control         */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: December 2020                         */
/* File: mpc_regulator.hpp                     */

#include <ros/ros.h>

//#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <string>

#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>

using namespace Eigen;

// MPC Wrapper Class
class IPodRegulator{
    public:
    // Constructor
    IPodRegulator(ros::NodeHandle& nh);

    // Destructor
    ~IPodRegulator(){;};

    private:
    // Attributes
    //States
    double pose_x, pose_y, pose_z;
    double Vx, Vy, Vz;
    double Qx, Qy, Qz, Qw; //quaternions to be converted in RPY
    //double Wx, Wy, Wz;
    double Wx_II, Wy_II, Wz_II;
    //double roll, pitch, yaw;
    double roll_II, pitch_II, yaw_II;
    double a, Va, b, Vb;

    //checks signals from sensor
    bool pose_check = false;
    bool twist_check = false;
    bool imu_check = false;

    Matrix<double, 16, 1> states;
    Matrix<double, 4, 1> inputs;
    Matrix<double, 4, 16> K_dlqr;

    Matrix<double, 3, 3> R_B_II;

    Matrix<double, 3, 1> RPY_II;
    Matrix<double, 3, 1> omega_II;


    Matrix<double, 3, 1> RPY;
    Matrix<double, 3, 1> omega;


    // Subscribers
    ros::Subscriber stateSub,
                    poseSub,
                    velSub,
                    imuSub;

    // Publishers
    ros::Publisher controlPub;

    // Timers
    ros::Timer controlTimer;

    // Messages
    mavros_msgs::State currentState;

    // Callbacks
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void sendControl(const ros::TimerEvent& e);
};