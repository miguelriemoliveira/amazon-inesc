#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise< geometry_msgs::Pose>("chatter", 1000);
    ros::Rate loop_rate(10);

    geometry_msgs::Pose p_;
    p_.position.x = 1.0;
    p_.position.y  = 0.5;
    p_.position.z  = 0.5;


    int count = 0;
    while (ros::ok())
    {
        p_.position.x += 1.0;
        if(  p_.position.x > 10.0)
            p_.position.x = 0;
        ROS_INFO("I send the pose");
        chatter_pub.publish(p_);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
// %EndTag(FULLTEXT)%
