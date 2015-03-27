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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <rospack/rospack.h>

using namespace std;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

pcl::PointCloud<PointT>::Ptr shelf (new pcl::PointCloud<PointT>);

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


        Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
        //(row, column)
        // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
        t(0,0) = n[0]; t(1,0) = n[1]; t(2,0) = n[2];
        t(0,1) = s[0]; t(1,1) = s[1]; t(2,1) = s[2];
        t(0,2) = a[0]; t(1,2) = a[1]; t(2,2) = a[2];

        // Define the translation 
        t(0,3) = ptO.x; t(1,3) = ptO.y; t(2,3) = ptO.z;

        Eigen::Matrix4f tinv = t.inverse();
        // Print the transformation
        //printf ("Method #1: using a Matrix4f\n");
        std::cout << t << std::endl;

        // Executing the transformation
        pcl::PointCloud<PointT>::Ptr transformed_shelf (new pcl::PointCloud<PointT> ());
        pcl::transformPointCloud (*shelf, *transformed_shelf, tinv);

        data->viewerPtr->addPointCloud<PointT> (transformed_shelf, "shelf_calibrated"); 
        data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5f, 1.0f, 0.5f, "shelf_calibrated");

        string file = "shelf_calibrated.pcd";
        cout << "Calibrated point cloud saved to " << file << endl;
        pcl::io::savePCDFileASCII (file, *transformed_shelf);
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

using namespace ros::package;
int main (int argc, char** argv)
{
    ros::init(argc, argv, "calibrate_reference_system_in_model_node");

     shelf = (pcl::PointCloud<PointT>::Ptr) new (pcl::PointCloud<PointT>);
    vector<int> indices;
    float theta = M_PI/20; // The angle of rotation in radians

    //Problem linking? check http://answers.ros.org/question/196935/roslib-reference-error/
    string path = ros::package::getPath("amazon_object_segmentation");

    //Load and remove NANs
    //if (pcl::io::loadPCDFile<PointT> ("shelf.pcd", *shelf) == -1) /[> load the file
    if (pcl::io::loadPCDFile<PointT> ("scene.pcd", *shelf) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    pcl::removeNaNFromPointCloud(*shelf, *shelf, indices);
    cout << "shelf pc has " << shelf->points.size() << " points" << endl;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //    Transform the shelf point cloud to make sure its not well aligned
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    //Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    ////Define a translation of 2.5 meters on the x axis.
    //transform_2.translation() << .1, 0.0, 0.0;
    //// The same rotation matrix as before; tetha radians arround Z axis
    //transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
    //// Print the transformation
    //printf ("\nMethod #2: using an Affine3f\n");
    //cout << transform_2.matrix() << endl;
    //// Executing the transformation
    //pcl::PointCloud<PointT>::Ptr transformed_shelf (new pcl::PointCloud<PointT> ());
    //pcl::transformPointCloud (*shelf, *transformed_shelf, transform_2);

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //    Run ICP
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //pcl::PointCloud<PointT>::Ptr Final (new pcl::PointCloud<PointT>);
    //pcl::IterativeClosestPoint<PointT, PointT> icp;
    //icp.setMaximumIterations (1000);
    //icp.setInputSource(transformed_shelf);
    //icp.setInputTarget(scene);
    //cout<< "Running icp ... wait a moment " << endl;
    //icp.align(*Final);
    //cout << "has converged:" << icp.hasConverged() << " score: " <<
    //icp.getFitnessScore() << endl;
    //cout << icp.getFinalTransformation() << endl;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //    Visualize
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    //viewer->setBackgroundColor (0, 0, 0);
    //viewer->addPointCloud<PointT> (scene, "scene"); 
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5f, 0.5f, 0.5f, "scene");

    viewer->addPointCloud<PointT> (shelf, "shelf"); 
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5f, 0.5f, 0.5f, "shelf");

    //viewer->addPointCloud<PointT> (Final, "shelf_aligned"); 
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1f, 0.9f, 0.1f, "shelf_aligned");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    //Add viewer callbacks:

    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d (new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback (pp_callback, (void*)&cb_args);
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&cb_args);
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

    cout << "Shift+click on three points [Origin, Origin + X, Origin + Y], then press 'c' to calibrate" << endl;

    //boost::share_ptr<pcl::visualization::PCLVisualization> p(new pcl::visualization::PCLVisualization("clouds")); 
    //int v1(0); 
    //int v2(0); 
    //p->createViewPort(0.0,0.0,0.5,1.0,v1);//create the first view port 
    //p->createViewPort(0.5,0.0,0.5,1.0,v2);//create the secondview port 
    //p->addPointCloud<PointT>(scene,"cloud1",v1);//show clouds in the first viewport with ID is cloud1 
    //p->addPointCloud<PointT>(shelf,"clouds2",v2); 

    //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //viewer.showCloud (scene);
    //viewer.addPointCloud<PointT>(scene);
    //viewer.addPointCloud<PointT>(shelf);
    //while (!viewer.wasStopped ())
    //{
    //}

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
}
