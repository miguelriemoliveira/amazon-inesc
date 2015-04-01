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

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <iostream>
#include <vector>
#include <ctime>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std;
using namespace ros;
using namespace ros::package;

pcl::PointCloud<PointT>::Ptr shelf;
string point_cloud_in_topic;
double time_to_wait_for_pointcloud;
string initial_arm_frame_id = "/world";
string final_arm_frame_id = "/wrist_3";
string camera_frame_id;
string shelf_frame_id = "/shelf";
bool calibrated = false;
tf::Transform t_camera_to_shelf;

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

unsigned int text_id = 0;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* args )
{
    struct callback_args* data = (struct callback_args *)args;

    if (event.getKeySym () == "c" && event.keyDown ())
    {

        if (data->clicked_points_3d->points.size() != 3)
        {
            std::cout << "Cannot calibrate, 3 points must be selected" << std::endl;
        }
        else
        {
            std::cout << "Starting calibration" << std::endl;
        }

        PointT ptO = data->clicked_points_3d->points[0];
        PointT ptX = data->clicked_points_3d->points[1];
        PointT ptY = data->clicked_points_3d->points[2];

        //Find the nsa vectors. the n vector will be the vector from pt1_projected to pt2_projected 
        double n[3] = {ptX.x - ptO.x, ptX.y - ptO.y, ptX.z - ptO.z};
        double s[3] = {ptY.x - ptO.x, ptY.y - ptO.y, ptY.z - ptO.z};

        //Find the nsa vectors. The s vector is given by the external product a*n
        double a[3]={n[1]*s[2] - n[2]*s[1],	n[2]*s[0] - n[0]*s[2], n[0]*s[1] - n[1]*s[0]};

        //Normalize all vectors
        normalize_vector(n); normalize_vector(s); normalize_vector(a);

        //*t = tf::Transform(tf::Matrix3x3(n[0], s[0] , a[0],
        //n[1], s[1] , a[1],  
        //n[2], s[2] , a[2]),  
        //tf::Vector3(centroid.x,
        //centroid.y,
        //centroid.z));


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
        t_camera_to_shelf.setOrigin(origin);
        t_camera_to_shelf.setRotation(tfqt);

        

        // Print the transformation
        //printf ("Method #1: using a Matrix4f\n");
        std::cout << t << std::endl;

        //Draw an X line in the shelf reference system
        pcl::PointCloud<pcl::PointXYZ>::Ptr camera_ref_system (new pcl::PointCloud<pcl::PointXYZ> ());
        //pcl::PointXYZ ptO(0,0,0);  pcl::PointXYZ ptX(1,0,0);
        //pcl::PointXYZ ptY(0,1,0);
        //pcl::PointXYZ ptZ(0,0,1);
        double scale=.3;
        camera_ref_system->points.push_back(pcl::PointXYZ(0,0,0));
        camera_ref_system->points.push_back(pcl::PointXYZ(scale,0,0));
        camera_ref_system->points.push_back(pcl::PointXYZ(0,scale,0));
        camera_ref_system->points.push_back(pcl::PointXYZ(0,0,scale));

        pcl::PointCloud<pcl::PointXYZ>::Ptr shelf_ref_system (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud (*camera_ref_system, *shelf_ref_system, t);

        //see http://www.pcl-users.org/line-width-for-addLine-in-visualizer-td4027508.html
        data->viewerPtr->addLine<pcl::PointXYZ> (shelf_ref_system->points[0], shelf_ref_system->points[1], 1.0 , 0.0, 0.0, "shelf_xaxis");
        data->viewerPtr->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10,"shelf_xaxis" );
        data->viewerPtr->addLine<pcl::PointXYZ> (shelf_ref_system->points[0], shelf_ref_system->points[2], 0.0 , 1.0, 0.0, "shelf_yaxis");
        data->viewerPtr->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10,"shelf_yaxis" );
        data->viewerPtr->addLine<pcl::PointXYZ> (shelf_ref_system->points[0], shelf_ref_system->points[3], 0.0 , 0.0, 1.0, "shelf_zaxis");
        data->viewerPtr->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10,"shelf_zaxis" );


        calibrated = true;

        // Executing the transformation
        //pcl::PointCloud<PointT>::Ptr transformed_shelf (new pcl::PointCloud<PointT> ());
        //pcl::transformPointCloud (*shelf, *transformed_shelf, tinv);

        //data->viewerPtr->addPointCloud<PointT> (transformed_shelf, "shelf_calibrated"); 
        //data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5f, 1.0f, 0.5f, "shelf_calibrated");

        //string file = "shelf_calibrated.pcd";
        //cout << "Calibrated point cloud saved to " << file << endl;
        //pcl::io::savePCDFileASCII (file, *transformed_shelf);
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

    shelf = (pcl::PointCloud<PointT>::Ptr) new (pcl::PointCloud<PointT>);
    vector<int> indices;

    //Problem linking? check http://answers.ros.org/question/196935/roslib-reference-error/
    string path = ros::package::getPath("amazon_object_segmentation");

    ros::Duration(0.1).sleep(); // sleep for half a second

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    ros::Duration(0.5).sleep(); // sleep for half a second

    //Receive point cloud
    point_cloud_in_topic = "/kinect/depth_registered/points";
    time_to_wait_for_pointcloud = 2.0; //secs

    cout << "Waiting for a point_cloud2 on topic " << point_cloud_in_topic << endl;
    sensor_msgs::PointCloud2::ConstPtr pcmsg = topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_in_topic, nh, ros::Duration(2.0));

    if (!pcmsg)
    {
        ROS_ERROR_STREAM("No point_cloud2 has been received after " << time_to_wait_for_pointcloud << "secs");
        return 0;
    }
    else
    {
        ROS_INFO_STREAM("Received point cloud");
    }

    //Set the camera frame_id
    camera_frame_id = pcmsg->header.frame_id;

    //convert from ros msg to pcl (already removes RGB component because IN is pcl::PointXYZ)
    pcl::fromROSMsg(*pcmsg, *shelf);
    //ros::Time t = pcmsg->header.stamp; //TODO does not work tf says time in the past!
    ros::Time t = ros::Time::now();

    //listen to the transform

    ros::Duration(0.1).sleep(); // sleep for half a second


/* _________________________________
  |                                 |
  |           PCL EXAMPLE           |
  |_________________________________| */


    //PCL example

  // Octree resolution - side length of octree voxels
  //float resolution = 32.0f;
  float resolution = 64.0f;

  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ> );

  //// Generate pointcloud data for cloudA
  //cloudA->width = 128;
  //cloudA->height = 1;
  //cloudA->points.resize (cloudA->width * cloudA->height);

  //for (size_t i = 0; i < cloudA->points.size (); ++i)
  //{
    //cloudA->points[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    //cloudA->points[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    //cloudA->points[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
  //}

  // Add points from cloudA to octree
  octree.setInputCloud (shelf);
  octree.addPointsFromInputCloud ();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  octree.switchBuffers ();


    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ> );
   
  //// Generate pointcloud data for cloudB 
  //cloudB->width = 128;
  //cloudB->height = 1;
  //cloudB->points.resize (cloudB->width * cloudB->height);

  //for (size_t i = 0; i < cloudB->points.size (); ++i)
  //{
    //cloudB->points[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    //cloudB->points[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    //cloudB->points[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
  //}
    cout << "Take the box away"<< endl;

    ros::Duration(5.0).sleep(); // sleep for half a second

    cout << "Waiting for a second point_cloud2 on topic " << point_cloud_in_topic << endl;
    pcmsg = topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_in_topic, nh, ros::Duration(2.0));

    if (!pcmsg)
    {
        ROS_ERROR_STREAM("No point_cloud2 has been received after " << time_to_wait_for_pointcloud << "secs");
        return 0;
    }
    else
    {
        ROS_INFO_STREAM("Received point cloud");
    }


    pcl::fromROSMsg(*pcmsg, *cloudB);



  // Add points from cloudB to octree
  octree.setInputCloud (cloudB);
  octree.addPointsFromInputCloud ();

  std::vector<int> newPointIdxVector;

  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree.getPointIndicesFromNewVoxels (newPointIdxVector);

  // Output points
  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
  for (size_t i = 0; i < newPointIdxVector.size (); ++i)
    std::cout << i << "# Index:" << newPointIdxVector[i]
              << "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
              << cloudB->points[newPointIdxVector[i]].y << " "
              << cloudB->points[newPointIdxVector[i]].z << std::endl;

}
    
