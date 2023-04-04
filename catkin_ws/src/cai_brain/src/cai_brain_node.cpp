#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

double w_l, w_r;

void wl_callback(const std_msgs::Float32::ConstPtr &msg);
void wr_callback(const std_msgs::Float32::ConstPtr &msg);
void camera_callback(const sensor_msgs::ImageConstPtr &msg);

int main(int argc, char *argv[])
{
    ROS_INFO("[CaiBrain] Starting node...");
    ros::init(argc, argv, "cai_brain_node");

    w_l = 0;
    w_r = 0;

    ros::NodeHandle nh;
    ros::Subscriber sub_wl = nh.subscribe("/wl", 10, wl_callback);
    ros::Subscriber sub_wr = nh.subscribe("/wr", 10, wr_callback);
    ros::Subscriber sub = nh.subscribe("/camera/image_raw", 10, camera_callback);
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

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

void camera_callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat image = cv_ptr->image;
    cv::imshow("Camera Image", image);
    cv::waitKey(1);
}
