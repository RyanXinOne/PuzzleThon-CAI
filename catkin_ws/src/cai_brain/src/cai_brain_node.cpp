#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

double w_l, w_r;

void wl_callback(const std_msgs::Float32::ConstPtr &msg);
void wr_callback(const std_msgs::Float32::ConstPtr &msg);

int main(int argc, char *argv[])
{
    ROS_INFO("[CaiBrain] Starting node...");
    ros::init(argc, argv, "cai_brain_node");

    w_l = 0;
    w_r = 0;

    ros::NodeHandle nh;
    ros::Subscriber sub_wl = nh.subscribe("wl", 10, wl_callback);
    ros::Subscriber sub_wr = nh.subscribe("wr", 10, wr_callback);
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::Rate loop_rate(100);
    double v = 0.5;
    double w = 3;
    while (ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::Twist msg_cmd_vel;
        msg_cmd_vel.linear.x = v;
        msg_cmd_vel.angular.z = w;
        pub_cmd_vel.publish(msg_cmd_vel);

        loop_rate.sleep();
    }

    return 0;
}

void wl_callback(const std_msgs::Float32::ConstPtr &msg)
{
    w_l = msg->data;
}

void wr_callback(const std_msgs::Float32::ConstPtr &msg)
{
    w_r = msg->data;
}
