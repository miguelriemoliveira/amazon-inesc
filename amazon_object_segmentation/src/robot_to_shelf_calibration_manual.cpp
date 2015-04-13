#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h> //hydro

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <rospack/rospack.h>
#include <sensor_msgs/PointCloud2.h>
//#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//#include <tf/Quaternion.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std;
using namespace ros;
using namespace ros::package;

pcl::PointCloud<PointT>::Ptr shelf;
pcl::PointCloud<PointT>::Ptr robot_calibration_pts;
string point_cloud_in_topic;
double time_to_wait_for_pointcloud;
string initial_arm_frame_id = "/world";
string final_arm_frame_id = "/wrist_3_link";
string camera_frame_id;
string shelf_frame_id = "/shelf";
bool calibrated = false;
tf::Transform t_camera_to_shelf;
tf::Transform t_world_to_shelf;
boost::shared_ptr<tf::TransformListener> listener;

/**
 * @brief Normalizes a vector
 * @param v the vector to normalize v[0]=x , v[1]=y, v[2]=z
 */
void normalize_vector(double *v)
{
    double n = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    v[0] = v[0]/n;
    v[1] = v[1]/n;
    v[2] = v[2]/n;
}


void compute_tf_from_three_points(tf::Transform* t_out, PointCloudT::Ptr pc)
{
    PointT ptO = pc->points[0];
    PointT ptX = pc->points[1];
    PointT ptY = pc->points[2];

    //Find the nsa vectors. the n vector will be the vector from pt1_projected to pt2_projected 
    double n[3] = {ptX.x - ptO.x, ptX.y - ptO.y, ptX.z - ptO.z};
    double s[3] = {ptY.x - ptO.x, ptY.y - ptO.y, ptY.z - ptO.z};
    double a[3]={n[1]*s[2] - n[2]*s[1],	n[2]*s[0] - n[0]*s[2], n[0]*s[1] - n[1]*s[0]};

    //Normalize all vectors
    normalize_vector(n); normalize_vector(s); normalize_vector(a);

    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    //(row, column)
    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    t(0,0) = n[0]; t(1,0) = n[1]; t(2,0) = n[2];
    t(0,1) = s[0]; t(1,1) = s[1]; t(2,1) = s[2];
    t(0,2) = a[0]; t(1,2) = a[1]; t(2,2) = a[2];

    // Define the translation 
    t(0,3) = ptO.x; t(1,3) = ptO.y; t(2,3) = ptO.z;

    Eigen::Matrix4d tinv = t.inverse();

    //Convert eigen to tf to publish
    tf::Vector3 origin;
    origin.setValue((t(0,3)),(t(1,3)),(t(2,3)));
    tf::Matrix3x3 tf3d;
    tf3d.setValue((t(0,0)), (t(0,1)), (t(0,2)), 
            (t(1,0)), (t(1,1)), (t(1,2)), 
            (t(2,0)), (t(2,1)), (t(2,2)));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);
    t_out->setOrigin(origin);
    t_out->setRotation(tfqt);

    // Print the transformation
    std::cout << t << std::endl;
}


struct callback_args{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *)args;
    if (event.getPointIndex () == -1)
        return;

    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);
    //Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");

}

unsigned int text_id = 0;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* args )
{
    struct callback_args* data = (struct callback_args *)args;

    if (event.getKeySym () == "c" && event.keyDown ())
    {

        if (data->clicked_points_3d->points.size() != 3)
        {
            std::cout << "Cannot calibrate, 3 points must be selected" << std::endl;
            return;
        }
        else
        {
            std::cout << "Starting calibration" << std::endl;
        }

        compute_tf_from_three_points(&t_camera_to_shelf, data->clicked_points_3d);

    ros::Time t = ros::Time::now();
    std::string error_msg;

    if (!listener->waitForTransform("kinect_rgb_optical_frame", "/world",
            t,
            ros::Duration(1),
            ros::Duration(0.01),
            &error_msg 
            ))
    {
        ROS_WARN("Warning 1: %s", error_msg.c_str());
        return;
    }

    ROS_INFO("Transform available");

    tf::StampedTransform transform_camera_to_world;
    try
    {
        listener->lookupTransform("kinect_rgb_optical_frame", "/world", t, transform_camera_to_world);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    ROS_INFO("Received transform ");

    t_world_to_shelf = t_camera_to_shelf.inverse() * transform_camera_to_world;
    t_world_to_shelf  = t_world_to_shelf.inverse();

    calibrated = true;

    }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
        void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
            event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

        char str[512];
        sprintf (str, "text#%03d", text_id++);
        if (text_id ==1)
        {
            viewer->addText("Origin", event.getX (), event.getY (), str);
        }
        else if (text_id ==2)
        {
            viewer->addText("X Axis", event.getX (), event.getY (), str);
        }
        else if (text_id ==3)
        {
            viewer->addText("Y Axis", event.getX (), event.getY (), str);
        }
    }
}




int main (int argc, char** argv)
{
    ros::init(argc, argv, "calibrate_reference_system_in_model_node");
    ros::NodeHandle nh;

    tf::TransformBroadcaster broadcaster;
    listener = (boost::shared_ptr<tf::TransformListener>) new (tf::TransformListener);

    shelf = (pcl::PointCloud<PointT>::Ptr) new (pcl::PointCloud<PointT>);
    vector<int> indices;

    //Problem linking? check http://answers.ros.org/question/196935/roslib-reference-error/
    string path = ros::package::getPath("amazon_object_segmentation");

    ros::Duration(1).sleep(); // sleep for half a second

    //Receive point cloud
    point_cloud_in_topic = "/kinect/depth_registered/points";
    time_to_wait_for_pointcloud = 1.0; //secs

    cout << "Waiting for a point_cloud2 on topic " << point_cloud_in_topic << endl;
    sensor_msgs::PointCloud2::ConstPtr pcmsg = topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_in_topic, nh, ros::Duration(time_to_wait_for_pointcloud));

    ros::spinOnce();
    if (!pcmsg)
    {
        ROS_ERROR_STREAM("No point_cloud2 has been received after " << time_to_wait_for_pointcloud << "secs");
        return 0;
    }
    else
    {
        ROS_INFO_STREAM("Received point cloud");
    }
    //ros::Time t = pcmsg->header.stamp; //TODO does not work tf says time in the past!
    ros::Time t = ros::Time::now();
    //ros::Time t = ros::Time(0);


    //Set the camera frame_id
    camera_frame_id = pcmsg->header.frame_id;

    //convert from ros msg to pcl (already removes RGB component because IN is pcl::PointXYZ)
    pcl::fromROSMsg(*pcmsg, *shelf);

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //    Visualize
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(shelf);
    viewer->addPointCloud<PointT> (shelf, rgb, "pc"); 
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();

    viewer->setBackgroundColor(1, 1, 1); 
    viewer->setCameraPosition(0.0497412, -0.196849, -0.0978747, 0.0427789, -0.185814, 0.0496169, -0.0956887, -0.992963, 0.0697719); 
    viewer->setCameraFieldOfView(0.523599); 
    viewer->setCameraClipDistances(1.48244,5.11656); 
    viewer->setPosition(1650, 152); 
    viewer->setSize(631, 491); viewer->updateCamera(); 

    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d (new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback (pp_callback, (void*)&cb_args);
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&cb_args);
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

    cout << "Shift+click on three points [Origin, Origin + X, Origin + Y], then press 'c' to calibrate" << endl;


    //Wait while the viewer is running
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        ros::spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

        if (calibrated == true)
        {
            ROS_INFO("Publishing transform from kinect to shelf");
            broadcaster.sendTransform(tf::StampedTransform(t_world_to_shelf, ros::Time::now(), "/world", "/shelf"));

        }

    }

    return (0);
}
