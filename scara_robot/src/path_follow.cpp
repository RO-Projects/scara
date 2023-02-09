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
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"


/*-------- DEFINE --------*/
// ROS Node parameters
#define QUEUE_SIZE 1
#define FREQUENCY 200.0 //float type



#define Ts (1/FREQUENCY)

/*-------- GLOBAL VARIABLES --------*/
// desired positions

geometry_msgs::Point act_pos;
geometry_msgs::Point traj[5];
geometry_msgs::Vector3 act_vel;

float arm_dist_rot = 0.0;
float scale_x_1 = 0.0;
float scale_x_2 = 0.0;

/*-------- FUNCTION SIGNATURES --------*/


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

    int i = 0;
    int j = 0;
    float err = 0.0;



    act_pos.x = arm_dist_rot*(scale_x_1 + scale_x_2);
    act_pos.y = 0.0;
    act_pos.z = 0.0;

    act_vel.x = 0.0;
    act_vel.y = 0.0;
    act_vel.z = 0.0;

    traj[0].x = 0.2;
    traj[0].y = 0.2;
    traj[0].z = 0.1;

    traj[1].x = 0.3;
    traj[1].y = 0.4;
    traj[1].z = 0.2;

    traj[2].x = 0.5;
    traj[2].y = 0.4;
    traj[2].z = 0.3;

    traj[3].x = 0.7;
    traj[3].y = 0.1;
    traj[3].z = 0.4;

    traj[4].x = 0.5;
    traj[4].y = 0.3;
    traj[4].z = 0.1;    
    
    // --- Communication --- //
    //Pub Objects
    ros::Publisher pos_pub = node_obj.advertise<geometry_msgs::Point>("/des_pos", QUEUE_SIZE);
    //Sub Objects
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
        
        //publish joint angles and z position

        j = i%5;
        pos_pub.publish(traj[j]);  

        err = sqrt( pow(traj[j].x - act_pos.x, 2) + pow(traj[j].y - act_pos.y, 2) + pow(traj[j].z - act_pos.z, 2));
    
        if (err < 0.0005) 
        {
           //ros::Duration(1.0).sleep();
            i++;
        }
        // Loop Rate
        loop_rate.sleep();
    }

    return 0;
}