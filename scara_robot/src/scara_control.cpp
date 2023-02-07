/**********MODEL OF UNICYCLE*********
 * This node simulate the kinematic *
 * model of unicycle.               *
 ************************************/

/*-------- INCLUDE --------*/
#include "math.h"
#include "stdio.h"

// ROS basics library
#include "ros/ros.h"

// ROS msgs
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"


/*-------- DEFINE --------*/
// ROS Node parameters
#define QUEUE_SIZE 10
#define FREQUENCY 10.0 //float type

// Init Diff. Equation
#define L1 0.4 //effective length of joint 1
#define L2 0.25 //effective length of joint 2



#define Ts (1/FREQUENCY)

/*-------- GLOBAL VARIABLES --------*/
// desired positions
geometry_msgs::Point des_pos;
std_msgs::Float64 theta1;
std_msgs::Float64 theta2;
std_msgs::Float64 z_des;
float _theta1;
float _theta2;

/*-------- FUNCTION SIGNATURES --------*/
void joint_angles(geometry_msgs::Point pos, float &theta1, float &theta2);

/*-------- CALLBACKS --------*/
void POScallBack(const geometry_msgs::Point::ConstPtr& msg)
{
    // Update State Space
    des_pos.x = msg->x;
    des_pos.y = msg->y;
    des_pos.z = msg->z;

    
    joint_angles(des_pos, _theta1, _theta2); 
    theta1.data = _theta1;
    theta2.data = _theta2;
    z_des.data = des_pos.z;
      
    
    ROS_INFO("x: %f | y: %f", theta1.data, theta2.data);
}

/*-------- MAIN --------*/
int main(int argc, char **argv)
{
    // --- Init node --- //
    ros::init(argc, argv, "scara_controller");
    ros::NodeHandle node_obj;
    des_pos.x = 0.0;
    des_pos.y = 0.0;
    des_pos.z = 0.0;
    theta1.data = 0.0;
    theta2.data = 0.0;
    _theta1 = 0.0;
    _theta2 = 0.0;
    




    // --- Communication --- //
    //Pub Objects
    ros::Publisher joint1 = node_obj.advertise<std_msgs::Float64>("/scara_robot/joint1_controller/command", QUEUE_SIZE);
    ros::Publisher joint2 = node_obj.advertise<std_msgs::Float64>("/scara_robot/joint2_controller/command", QUEUE_SIZE);
    ros::Publisher joint3 = node_obj.advertise<std_msgs::Float64>("/scara_robot/joint3_controller/command", QUEUE_SIZE);

    //Sub Objects
    ros::Subscriber sub_states = node_obj.subscribe("/des_pos", QUEUE_SIZE, POScallBack);

    // Screen Output:
    ROS_INFO("**************************");
    ROS_INFO("Control Node activated.");
    //ROS_INFO("Init Pose of Unicycle:");
    //ROS_INFO("[x0: %f, y0: %f, theta0: %f", X0, Y0, THETA0);
    ROS_INFO("**************************");

    // --- Loop Init --- //
    ros::Rate loop_rate(FREQUENCY);

    //node_obj.getParam("/vbar", vbar);
    //node_obj.getParam("/control_gain", K);




    // --- Main Loop --- //
    // while(ros::ok()) is essentially while(1), until the node crash.
    while(ros::ok())
    {
        // Read Control Input
        ros::spinOnce();
        
        //publish joint angles and z position
        joint1.publish(theta1);
        joint2.publish(theta2);
        joint3.publish(z_des);   
        

        // Loop Rate
        loop_rate.sleep();
    }

    return 0;
}



void joint_angles(geometry_msgs::Point pos, float &theta1, float &theta2)
{
    float l1 =  L1; //effective lenght of first scara arm
    float l2 = L2; //effective lenght of second scara arm
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