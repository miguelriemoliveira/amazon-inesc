#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

void chatterCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    ROS_INFO("I received a pose");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    ros::spin();

    return 0;
}
