#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>



geometry_msgs::Point gp_est_body;
geometry_msgs::Point gp_est_world;
geometry_msgs::PoseStamped current_pos;

std_msgs::Float64 gp_est_body_x;

std_msgs::Float64 gp_est_world_x;
std_msgs::Float64 gp_est_body_y;
std_msgs::Float64 gp_est_world_y;


std_msgs::Float64 gp_est_body_z;
std_msgs::Float64 gp_est_world_z;

//std::vector<double> current_pos;

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = *msg;
   // {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
   //                    msg->pose.orientation.x, msg->pose.orientation.y,
   //                    msg->pose.orientation.z, msg->pose.orientation.w};
}

void gp_est_body_x_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    gp_est_body.x = msg->data[0];
}


void gp_est_body_y_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    gp_est_body.y = msg->data[0];
}


void gp_est_body_z_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    gp_est_body.z = msg->data[0];
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_node");
    ros::NodeHandle nh;

    ros::Subscriber gp_est_body_sub_x = nh.subscribe<std_msgs::Float64MultiArray>
            ("/gp_disturb_reg/mu/x", 10, gp_est_body_x_cb);
    
    ros::Subscriber gp_est_body_sub_y = nh.subscribe<std_msgs::Float64MultiArray>
            ("/gp_disturb_reg/mu/y", 10, gp_est_body_y_cb);
    
    ros::Subscriber gp_est_body_sub_z = nh.subscribe<std_msgs::Float64MultiArray>
            ("/gp_disturb_reg/mu/z", 10, gp_est_body_z_cb);



    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 10, pos_cb);


    ros::Publisher gp_est_world_pub = nh.advertise<geometry_msgs::Point>
            ("/gp_disturb_reg_world", 10);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    while(ros::ok()){


        //current_pos.resize(7);
        tf::Quaternion mocap_q(current_pos.pose.orientation.x,
                               current_pos.pose.orientation.y,
                               current_pos.pose.orientation.z,
                               current_pos.pose.orientation.w);

        tf::Matrix3x3 R_BI(mocap_q);
        //R_BI = R_BI.transpose();
        

        std::cout << "current_pos " << current_pos<< "\n";

       // std::cout << "current_pos.at(3): " << current_pos[3]<< "\n";
       // std::cout << "current_pos.at(4): " << current_pos[4] << "\n";
       // std::cout << "current_pos.at(5): " << current_pos[5] << "\n";
       // std::cout << "current_pos.at(6): " << current_pos[6] << "\n";



        std::cout << "gp_est_body x: " << gp_est_body.x << "\n";
        std::cout << "gp_est_body y: " << gp_est_body.y << "\n";
        std::cout << "gp_est_body z: " << gp_est_body.z << "\n";


        std::cout << "R00: " << R_BI[0][0] << "\n";
        std::cout << "R10: " << R_BI[1][0] << "\n";
        std::cout << "R20: " << R_BI[2][0] << "\n";


        gp_est_world.x = R_BI[0][0] * gp_est_body.x + R_BI[0][1] * gp_est_body.y + R_BI[0][2] * gp_est_body.z;
        gp_est_world.y = R_BI[1][0] * gp_est_body.x + R_BI[1][1] * gp_est_body.y + R_BI[1][2] * gp_est_body.z;
        gp_est_world.z = R_BI[2][0] * gp_est_body.x + R_BI[2][1] * gp_est_body.y + R_BI[2][2] * gp_est_body.z;

        

        gp_est_world_pub.publish(gp_est_world);
        

       

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}