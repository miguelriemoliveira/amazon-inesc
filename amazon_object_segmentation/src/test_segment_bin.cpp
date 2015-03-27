#include <iostream>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <amazon_msgs/SegmentBinAction.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h> 

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


using namespace std;
using namespace ros;

string point_cloud_in_topic;
double time_to_wait_for_pointcloud;

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_segment_bin");
    ros::NodeHandle nh;

    int bin_number=0;
    param::get("~bin_number", bin_number);
    ROS_INFO("bin_number is %d",bin_number);


    tf::TransformListener listener;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/segmented_point_cloud_from_bin", 10);
    //create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<amazon_msgs::SegmentBinAction> ac("segment_point_cloud_from_bin", true);
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time



    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    //image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);


    time_to_wait_for_pointcloud = 2.0; //secs
    point_cloud_in_topic = "/kinect/depth_registered/points";

    ros::Duration(0.1).sleep(); // sleep for half a second

    cout << "Waiting for a point_cloud2 on topic " << point_cloud_in_topic << endl;
    sensor_msgs::PointCloud2::ConstPtr pcmsg = topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_in_topic, nh, ros::Duration(time_to_wait_for_pointcloud));

    if (!pcmsg)
    {
        ROS_ERROR_STREAM("No point_cloud2 has been received after " << time_to_wait_for_pointcloud << "secs");
        return 0;
    }
    else
    {
        ROS_INFO_STREAM("Received point cloud");
    }

    string image_message_topic = "/kinect/rgb/image_color";
    sensor_msgs::Image::ConstPtr image_msg = ros::topic::waitForMessage<sensor_msgs::Image>(image_message_topic);
    if (!image_msg)
    {
        ROS_ERROR_STREAM("No image has been received after " << time_to_wait_for_pointcloud << "secs");
        return 0;
    }
    else
    {
        ROS_INFO_STREAM("Received image message in topic " << image_message_topic);
    }


    cv_bridge::CvImagePtr image;
    try
    {
        image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return 0;
    }


    //cv::Mat image(200, 200, CV_8UC3, Scalar(0));
    //CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source,
    //const std::string& encoding = std::string());
    //try
    //{
    //cv::imshow("view", cv_bridge::toCvShare(image_msg, "bgr8")->image);
    //cv::waitKey(30);
    //}
    //catch (cv_bridge::Exception& e)
    //{
    //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
    //}

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    amazon_msgs::SegmentBinGoal goal;
    goal.bin_number = bin_number;
    goal.point_cloud = *pcmsg;
    goal.compute_roi = true;
    goal.camera_info_topic = "kinect/rgb/camera_info";
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished:%s",state.toString().c_str());

        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        pcl::PointCloud<PointT>::Ptr pc;
        pc = (pcl::PointCloud<PointT>::Ptr) new (pcl::PointCloud<PointT>);
        pcl::fromROSMsg(ac.getResult()->segmented_point_cloud, *pc);

        ROS_INFO("received point_cloud on frame_id %s", pc->header.frame_id.c_str());

        cv::Rect rect;
        rect.x = ac.getResult()->roi.x_offset;
        rect.y = ac.getResult()->roi.y_offset;
        rect.width = ac.getResult()->roi.width;
        rect.height = ac.getResult()->roi.height;
        rectangle(image->image, rect, CV_RGB(255,0,0));
        //{
            //cv::Point p1(ac.getResult()->roi.x_offset, ac.getResult()->roi.y_offset);
            //cv::Point p2(ac.getResult()->roi.x_offset + ac.getResult()->roi.width, ac.getResult()->roi.y_offset);
            //cv::line(image->image, p1, p2, CV_RGB(255,0,0));
        //}

        //{
            //cv::Point p1(ac.getResult()->roi.x_offset, ac.getResult()->roi.y_offset);
            //cv::Point p2(ac.getResult()->roi.x_offset + ac.getResult()->roi.width, ac.getResult()->roi.y_offset);
            //cv::line(image->image, p1, p2, CV_RGB(255,0,0));
        //}
        cv::imshow("view", image->image);

        cv::waitKey(30);


        visualization_msgs::Marker m;
        m.ns = "segmented_point_cloud";
        m.header.frame_id = pc->header.frame_id;
        m.header.stamp = ros::Time::now();
        m.action = visualization_msgs::Marker::ADD;
        m.id = 0;
        m.type = visualization_msgs::Marker::POINTS;
        m.scale.x = m.scale.y = .005;

        // Color
        m.color.r = 1.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 1.0;

        geometry_msgs::Point p;
        for (size_t i=0; i < pc->points.size(); i++)
        {
            p.x = pc->points[i].x;
            p.y = pc->points[i].y;
            p.z = pc->points[i].z;
            m.points.push_back(p);
        }
        marker_pub.publish(m);

        ros::Rate loop_rate(20);
        while (ros::ok())
        {

            ros::spinOnce();
            loop_rate.sleep();

            m.header.stamp = ros::Time::now();
            marker_pub.publish(m);
        }




    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
    }

    //exit
    return 0;
}
