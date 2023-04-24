#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

double V_INI = 0.05;
double W_INI = 0;
double K_P = 2;

double v, w;
double w_l, w_r;

void wl_callback(const std_msgs::Float32::ConstPtr &msg);
void wr_callback(const std_msgs::Float32::ConstPtr &msg);
void camera_callback(const sensor_msgs::ImageConstPtr &msg);
double locate_middle_lane(cv::Mat &image);

int main(int argc, char *argv[])
{
    ROS_INFO("[CaiBrain] Starting node...");
    ros::init(argc, argv, "cai_brain_node");

    v = V_INI;
    w = W_INI;
    w_l = 0;
    w_r = 0;

    ros::NodeHandle nh;
    ros::Subscriber sub_wl = nh.subscribe("/wl", 10, wl_callback);
    ros::Subscriber sub_wr = nh.subscribe("/wr", 10, wr_callback);
    ros::Subscriber sub = nh.subscribe("/camera/image_raw", 10, camera_callback);
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    ros::Rate loop_rate(10);
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
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    // central of white region
    double mid_lane = locate_middle_lane(image);
    ROS_INFO("middle lane: %lf", mid_lane);
    if (mid_lane > -1)
    {
        if (mid_lane >= 0.47 && mid_lane <= 0.53){
            v = 0.2;
        }
        else{
            v = V_INI;
        }
        double error = mid_lane - 0.5;
        w = -K_P * error;
    }

    cv::imshow("camera", image);
    cv::waitKey(1);
}

// find middle lane position [0, 1] in input image
double locate_middle_lane(cv::Mat &image)
{
    int margin = image.cols / 5;
    int threshold = 150;

    vector<int> pixel_candidates;
    for (int i = margin; i < image.cols - margin; i++)
    {
        // last row as the scan line
        if (image.at<uchar>(image.rows - 1, i) > threshold)
        {
            pixel_candidates.push_back(i);
        }
    }
    // return middle candidates
    return !pixel_candidates.empty() ? pixel_candidates[pixel_candidates.size() / 2] / (double)(image.cols - 1) : -1;
}