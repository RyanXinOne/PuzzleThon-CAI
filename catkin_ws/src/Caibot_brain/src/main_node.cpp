#include <ros/ros.h>
#include <std_msgs/Float32.h>

void cai_Callback(const std_msgs::Float32::ConstPtr &msg)
{
    ROS_INFO("cai: %f", msg->data);
}

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("wr", 10, cai_Callback);

    while (ros::ok())
    {
        /* code */
        ros::spinOnce();
    }

    return 0;
}
