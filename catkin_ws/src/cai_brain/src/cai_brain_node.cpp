#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

double w_l, w_r;
double v = 0.05, w = 0;

void wl_callback(const std_msgs::Float32::ConstPtr &msg);
void wr_callback(const std_msgs::Float32::ConstPtr &msg);
void camera_callback(const sensor_msgs::ImageConstPtr &msg);
int find_line(cv::Mat image);

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

    // to grey
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    // central of white region
    int center = find_line(image);

    ROS_INFO("center: %d", center);

    if (center < image.cols / 2)
    {
        w = 0.15;
    }
    else
    {
        w = -0.15;
    }
    cv::imshow("result", image);

    cv::waitKey(1);
}

// define a function, input a image and output an int
int find_line(cv::Mat image){
    // check the white region at the bottom line
    int left = 0;
    int right = 0;
    bool is_left = false;
    for (int i = image.cols / 4; i < image.cols * 3 / 4; i++)
    {
        if (image.at<uchar>(image.rows - 1, i) >= 127 && !is_left)
        {
            left = i;
            is_left = true;
            // break;
        }
        if (image.at<uchar>(image.rows - 1, i) < 127 && is_left)
        {
            right = i-1;
            break;
        }
    }
    return (left+right)/2;
}