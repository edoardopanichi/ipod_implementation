/* Quadcopter Model Predictive Control         */
/* Author: Edoardo Panichi                  */
/* E-Mail: edoardo.panichi@studio.unibo.it         */
/* Date: July 2021                         */
/* File: mpc_regulator.cpp                     */

#include <ipod_regulator/ipod_regulator.hpp>

IPodRegulator::IPodRegulator(ros::NodeHandle& nh):
    stateSub(nh.subscribe("/mavros/state", 1, &IPodRegulator::stateCallback, this)),
    poseSub(nh.subscribe("/mavros/local_position/pose", 1, &IPodRegulator::positionCallback, this)),
    velSub(nh.subscribe("/mavros/local_position/velocity_local", 1, &IPodRegulator::velocityCallback, this)),
    imuSub(nh.subscribe("/mavros/imu/data", 1, &IPodRegulator::imuCallback, this)),
    controlPub(nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1)),
    controlTimer(nh.createTimer(ros::Duration(0.004), &IPodRegulator::sendControl, this)){

    //nh.param("mpc/q_px", q_px, 200.0);
    K_dlqr << 3.78067347045174e-13,	7.32114633388227e-13,	2.33916331306561e-12,	4.53462274926990e-12,	-0.705611359746933,	-1.63868319275470,	1.12758132756011e-11,	-2.30827406101385e-12,	7.06497182809123e-13,	1.31271346252949e-13,	-2.76097172818821e-14,	1.77793205510050e-14,	8.25538036040993e-12,	2.55677400887911e-12,	4.53157990136421e-11,	1.43737426044072e-11,
                2.96842694445977e-12,	6.03816604896640e-12,	0.594718971974112,	1.50738421335288,	-2.93533479592037e-12,	-5.86706257605988e-12,	20.5127002757654,	-2.68108127530328e-11,	1.64436197133287e-12,	2.17279131629000,	-3.24237735159603e-13,	4.37064830120123e-14,	8.14844235062223e-11,	2.56770069026784e-11,	36.3358435251872,	11.7295605649480,
                -0.595554139019894,	-1.50957075054796,	-2.01865949984761e-12,	-4.02751279258454e-12,	1.48151420487736e-12,	1.21437397172454e-12,	-2.63337097882559e-11,	20.5529667806888,	-2.94510595870818e-13,	-3.23465631658797e-13,	2.17815670938260,	-2.39156967090314e-15,	-36.4000638914474,	-11.7501987862378,	-6.63166881205161e-11,	-2.02525466188711e-11,
                1.90530787304951e-13,	2.61523347261721e-13,	2.54692780648377e-13,	5.95431947819423e-13,	-2.15080691036392e-13,	-4.52712261441151e-13,	1.97369405711414e-12,    1.06844105935883e-13,	2.52079341976229,	2.40903440131138e-14,	1.65682056632322e-15,	2.12512827612543,	1.45901354541794e-12,	4.19317493276057e-13,	6.44365031199446e-12,	2.05427879165618e-12;

    a, Va, b, Vb = 0;

    //Rotation matrix from the body ref frame of the simulator to mine 
    R_B_II << 0,    1,    0,
              1,    0,    0,
              0,    0,    -1;  

    std::cout << K_dlqr << std::endl;
}

// Timer Callback
void IPodRegulator::sendControl(const ros::TimerEvent& e){
    //check if we get signals from the topics
    if (!pose_check || !twist_check || !imu_check) {  
        std::cout << "no signal" << std::endl;
        return;
    }

    states << pose_x, Vx, pose_y, Vy, pose_z - 2, Vz, RPY(0), RPY(1), RPY(2), omega(0), omega(1), omega(2), a, Va, b, Vb;

    inputs = -K_dlqr*states;
    std::cout << inputs(0) << std::endl;

    mavros_msgs::ActuatorControl ctrl_action;
    ctrl_action.controls[0] = inputs(1)/150;
    ctrl_action.controls[1] = inputs(2)/150;
    ctrl_action.controls[2] = inputs(3)/150;
    ctrl_action.controls[3] = -(inputs(0) - 9.81*1.55)/25.5;
    ctrl_action.group_mix = 0;

    controlPub.publish(ctrl_action);

}

// Callback Functions
void IPodRegulator::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_x = msg->pose.position.x;
    pose_y = msg->pose.position.y;
    pose_z = msg->pose.position.z;

    pose_check = true;
}

void IPodRegulator::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    Vx = msg->twist.linear.x;
    Vy = msg->twist.linear.y;
    Vz = msg->twist.linear.z;

    twist_check = true;
}

void IPodRegulator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    Qx = msg->orientation.x;
    Qy = msg->orientation.y;
    Qz = msg->orientation.z;
    Qw = msg->orientation.w;

     // roll (x-axis rotation)
    double sinr_cosp = 2 * (Qw * Qx + Qy * Qz);
    double cosr_cosp = 1 - 2 * (Qx * Qx + Qy * Qy);
    roll_II = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (Qw * Qy - Qz * Qx);
    if (std::abs(sinp) >= 1)
        pitch_II = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch_II = std::asin(sinp);

    //pitch += M_PI;    

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (Qw * Qz + Qx * Qy);
    double cosy_cosp = 1 - 2 * (Qy * Qy + Qz * Qz);
    yaw_II = std::atan2(siny_cosp, cosy_cosp);

    RPY_II << roll_II, pitch_II, yaw_II;

    RPY << R_B_II * RPY_II; 

    //yaw += M_PI/2;

    //print the values of RPY
    //std::cout << roll << " " << pitch << " " << yaw << " " << std::endl;

    Wx_II = msg->angular_velocity.x;
    Wy_II = msg->angular_velocity.y;
    Wz_II = msg->angular_velocity.z;

    omega_II << Wx_II, Wy_II, Wz_II;

    omega << R_B_II * omega_II; 

    imu_check = true;
}

void IPodRegulator::stateCallback(const mavros_msgs::State::ConstPtr& msg){
    currentState = *msg;
    return;
}
