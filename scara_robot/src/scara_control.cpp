/**********MODEL OF UNICYCLE*********
 * This node simulate the kinematic *
 * model of unicycle.               *
 ************************************/

/*-------- INCLUDE --------*/
#include "math.h"
#include "stdio.h"
#include "/usr/include/eigen3/Eigen/Eigen"
// ROS basics library
#include "ros/ros.h"

// ROS msgs
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

using namespace Eigen;
/*-------- DEFINE --------*/
// ROS Node parameters
#define QUEUE_SIZE 1
#define FREQUENCY 200.0 //float type



#define Ts (1/FREQUENCY)

/*-------- GLOBAL VARIABLES --------*/
// desired positions

geometry_msgs::Point act_pos;
geometry_msgs::Point des_pos;
geometry_msgs::Vector3 act_vel;

std_msgs::Float64 theta1;
std_msgs::Float64 theta2;
std_msgs::Float64 z_des;

float _theta1;
float _theta2;
float _z;

float _theta1_des;
float _theta2_des;
float _z_des;

float arm_dist_rot = 0.0;
float scale_x_1 = 0.0;
float scale_x_2 = 0.0;
float imu_freq = 0.0;
float T_imu = 0.0;
Vector3d q_prev(0,0,0);

/*-------- FUNCTION SIGNATURES --------*/
void clik(geometry_msgs::Point pos, float &theta1, float &theta2, float &z_des);
void joint_angles(geometry_msgs::Point pos, float &theta1, float &theta2);

/*-------- CALLBACKS --------*/
void DESPOScallBack(const geometry_msgs::Point::ConstPtr& msg)
{
    // Update State Space
    des_pos.x = msg->x;
    des_pos.y = msg->y;
    des_pos.z = msg->z;

    /*
    joint_angles(des_pos, _theta1_des, _theta2_des); 
    theta1.data = _theta1_des;
    theta2.data = _theta2_des;
    z_des.data = des_pos.z; */
      
    
    //ROS_INFO("x: %f | y: %f", des_pos.x, des_pos.y);
}


void JOINTcallBack(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Update State Space
    _theta1 = msg->position[0];
    _theta2 = msg->position[1];     
    
    //ROS_INFO("Arm1: %f | Arm2: %f", msg->position[0], msg->position[1]);
} 
/*
void IMUcallBack(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Update State Space

    act_vel.x = act_vel.x + (msg->linear_acceleration.x)*Ts;
    act_pos.x = act_pos.x + act_vel.x * Ts;
      
    
    ROS_INFO("x: %f | a: %f", act_pos.x, msg->linear_acceleration.x);
} */
void POScallBack(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    // Update State Space
    act_pos = msg->pose[4].position; //joint 4 is bottom of vertical arm
      
    
    //ROS_INFO("x: %f | y: %f | z: %f", act_pos.x, act_pos.y, act_pos.z);
}

/*-------- MAIN --------*/
int main(int argc, char **argv)
{
    // --- Init node --- //
    ros::init(argc, argv, "scara_controller");
    ros::NodeHandle node_obj;

    node_obj.getParam("/kinematic_params/arm_dist_rot", arm_dist_rot);
    node_obj.getParam("/kinematic_params/scalex_1", scale_x_1);
    node_obj.getParam("/kinematic_params/scalex_2", scale_x_2);
    node_obj.getParam("/imu/freq", imu_freq);
    T_imu = 1/imu_freq;

    des_pos.x = arm_dist_rot*(scale_x_1 + scale_x_2);
    des_pos.y = 0.0;
    des_pos.z = 0.0;

    act_pos.x = arm_dist_rot*(scale_x_1 + scale_x_2);
    act_pos.y = 0.0;
    act_pos.z = 0.0;

    act_vel.x = 0.0;
    act_vel.y = 0.0;
    act_vel.z = 0.0;
    
    theta1.data = 0.0;
    theta2.data = 0.0;
    _theta1 = 0.0;
    _theta2 = 0.0;
    _theta1_des = 0.0;
    _theta2_des = 0.0;
    _z_des = 0.0;



    // --- Communication --- //
    //Pub Objects
    ros::Publisher joint1 = node_obj.advertise<std_msgs::Float64>("/scara_robot/joint1_controller/command", QUEUE_SIZE);
    ros::Publisher joint2 = node_obj.advertise<std_msgs::Float64>("/scara_robot/joint2_controller/command", QUEUE_SIZE);
    ros::Publisher joint3 = node_obj.advertise<std_msgs::Float64>("/scara_robot/joint3_controller/command", QUEUE_SIZE);

    //Sub Objects
    ros::Subscriber sub_states = node_obj.subscribe("/des_pos", QUEUE_SIZE, DESPOScallBack);
    ros::Subscriber joint_states = node_obj.subscribe("/scara_robot/joint_states", QUEUE_SIZE, JOINTcallBack);
    //ros::Subscriber imu_states = node_obj.subscribe("/scara_robot/imu", QUEUE_SIZE, IMUcallBack);
    ros::Subscriber imu_states = node_obj.subscribe("/gazebo/link_states", QUEUE_SIZE, POScallBack);
    
    
    // Screen Output:
    ROS_INFO("**************************");
    ROS_INFO("Control Node activated.");
    //ROS_INFO("Init Pose of Unicycle:");
    //ROS_INFO("[x0: %f, y0: %f, theta0: %f", X0, Y0, THETA0);
    ROS_INFO("**************************");

    // --- Loop Init --- //
    ros::Rate loop_rate(FREQUENCY);

    // --- Main Loop --- //
    // while(ros::ok()) is essentially while(1), until the node crash.
    while(ros::ok())
    {
        // Read Control Input
        ros::spinOnce();
        
        
        //perform CLIK algorithm
        clik(des_pos, _theta1_des, _theta2_des, _z_des);

        //fill the data-body of the messages to be published
        theta1.data = _theta1_des;
        theta2.data = _theta2_des;
        z_des.data = _z_des;

        //publish joint angles and z position
        joint1.publish(theta1);
        joint2.publish(theta2);
        joint3.publish(z_des);   
        

        // Loop Rate
        loop_rate.sleep();
    }

    return 0;
}

void clik(geometry_msgs::Point pos, float &theta1, float &theta2, float &z_des)
{
    float l1 =  scale_x_1*arm_dist_rot; //effective lenght of first scara arm
    float l2 = scale_x_2*arm_dist_rot; //effective lenght of second scara arm
    Vector3d err(pos.x - act_pos.x, pos.y - act_pos.y, pos.z - act_pos.z);

    Matrix3d jacob;
    jacob <<    (-l1*sin(_theta1)) - (l2*sin(_theta1+_theta2)), -l2*sin(_theta1+_theta2), 0,
                l1*cos(_theta1) + l2*cos(_theta1+_theta2), l2*cos(_theta1+_theta2), 0,
                0, 0, 1;
    Matrix3d jacob_pinv = jacob.transpose();//completeOrthogonalDecomposition().pseudoInverse();

    //Matrix3d I;
    //I.setIdentity();
    Matrix3d K;
    //Gain Matrix: K values for the revolute joint chosen higher
    K <<    7 ,0 ,0,
            0, 7, 0,
            0, 0, 3;

    Vector3d e = K*err;
    Vector3d q_dot = jacob_pinv * e;
    Vector3d q = q_dot*Ts + q_prev;
    q_prev = q;
    
    theta1 = q(0);
    theta2 = q(1);
    z_des = q(2);

    ROS_INFO("err0: %f | err1: %f | err2: %f", err(0), err(1), err(2));

}


void joint_angles(geometry_msgs::Point pos, float &theta1, float &theta2)
{
    float l1 =  scale_x_1*arm_dist_rot; //effective lenght of first scara arm
    float l2 = scale_x_2*arm_dist_rot; //effective lenght of second scara arm
    float x_des = pos.x;
    float y_des = pos.y; 

    float r1 = sqrt((x_des * x_des) + (y_des * y_des));
 
    // ϕ1 = arccos((l2^2 - r1^2 - l1^2)/(-2*r1*l1))
    // returned value is in range [0, pi] rad
    float phi_1 = acos(((l2 * l2) - (r1 * r1) - (l1 * l1))/(-2.0 * r1 * l1));
 
    // ϕ2 = arctan((y_0_2) / (x_0_2)) 
    // returned value is in range [-pi, +pi] rad
    float phi_2 = atan2(y_des,x_des);
 
    // θ1 =  ϕ1 - ϕ2
    theta1 = phi_2 - phi_1;
 
    // ϕ3 = arccos((r1^2 - l1^2 - l2^2)/(-2*l1*l2))
    float phi_3 = acos(((r1 * r1) - (l1 * l1) - (l2 * l2))/(-2.0 * l1 * l2));
 
    //θ2 = 180° - ϕ3 
    theta2 = M_PI - phi_3;

}