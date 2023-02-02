/**********MODEL OF UNICYCLE*********
 * This node simulate the kinematic *
 * model of unicycle.               *
 ************************************/

/*-------- INCLUDE --------*/
// ROS basics library
#include "ros/ros.h"

// ROS msgs
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

/*-------- DEFINE --------*/
// ROS Node parameters
#define QUEUE_SIZE 10
#define FREQUENCY 1000.0 //float type

// Init Diff. Equation
#define X0 0.0
#define Y0 10.0
#define THETA0 0.0



#define Ts (1/FREQUENCY)

/*-------- GLOBAL VARIABLES --------*/
// State Space
geometry_msgs::Pose2D state_space;
geometry_msgs::Twist control_vel;


float v;
float w;
int vbar = 0;
int K = 0;

/*-------- FUNCTION SIGNATURES --------*/
geometry_msgs::Twist controler(geometry_msgs::Pose2D q_prev);
float my_sinc(float angle);

/*-------- CALLBACKS --------*/
void IMUcallBack(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Update State Space
    state_space.x = msg->x;
    state_space.y = msg->y;
    state_space.theta = msg->theta;
    
    
    ROS_INFO("y: %f", state_space.y);
}

/*-------- MAIN --------*/
int main(int argc, char **argv)
{
    // --- Init node --- //
    ros::init(argc, argv, "scara_controller");
    ros::NodeHandle node_obj;
    control_vel.linear.x = 0.0;
    control_vel.linear.y = 0.0;
    control_vel.angular.z = 0.0;

    // --- Communication --- //
    //Pub Objects
    ros::Publisher joint1 = node_obj.advertise<std_msgs::Float64>("/joint1_controller/command", QUEUE_SIZE);

    //Sub Objects
    ros::Subscriber sub_imu = node_obj.subscribe("/imu", QUEUE_SIZE, IMUcallBack);
    ros::Subscriber sub_states = node_obj.subscribe("/joint_states", QUEUE_SIZE, STATEScallBack);

    // Screen Output:
    ROS_INFO("**************************");
    ROS_INFO("Control Node activated.");
    //ROS_INFO("Init Pose of Unicycle:");
    //ROS_INFO("[x0: %f, y0: %f, theta0: %f", X0, Y0, THETA0);
    ROS_INFO("**************************");

    // --- Loop Init --- //
    ros::Rate loop_rate(FREQUENCY);

    node_obj.getParam("/vbar", vbar);
    node_obj.getParam("/control_gain", K);




    // --- Main Loop --- //
    // while(ros::ok()) is essentially while(1), until the node crash.
    while(ros::ok())
    {
        // Read Control Input
        ros::spinOnce();

        // Compute Kinematics
        // TC: q_dot(t) = S(q) * ni(t)
        // TD: (Euler1) q(k+1) = q(k) + Ts*S(q(k))*ni(k)
        control_vel = controler(state_space);

        // Publish updated state space
        joint1.publish(control_vel);
        joint2.publish(control_vel);

        // Loop Rate
        loop_rate.sleep();
    }

    return 0;
}

geometry_msgs::Twist controler(geometry_msgs::Pose2D q_prev)
{
    /************************************************  
     *  This function is voluntarily not optimized  *
     *  for educational purpose.                    *
     ************************************************/
    
    // Define q(k+1)
    geometry_msgs::Twist vel;

    // Compute Kinematics
    vel.linear.x = vbar;              //q_prev.x + Ts*cos(q_prev.theta)*vel;
    vel.angular.z = -q_prev.y * vbar * my_sinc(q_prev.theta) - K*q_prev.theta;              //q_prev.y + Ts*sin(q_prev.theta)*vel;
    

    return vel;
}

float my_sinc(float angle)
{
    return angle == 0 ? 1 : sin(angle)/angle;
}