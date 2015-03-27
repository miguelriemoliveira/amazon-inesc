
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


using namespace std;

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr shelf (new pcl::PointCloud<pcl::PointXYZ>);
    vector<int> indices;
    float theta = M_PI/20; // The angle of rotation in radians

    //Load and remove NANs
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("scene.pcd", *scene) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    pcl::removeNaNFromPointCloud(*scene, *scene, indices);
    cout << "scene pc has " << scene->points.size() << " points" << endl;


    //Load and remove NANs
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("shelf.pcd", *shelf) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    pcl::removeNaNFromPointCloud(*shelf, *shelf, indices);
    cout << "shelf pc has " << shelf->points.size() << " points" << endl;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //    Transform the shelf point cloud to make sure its not well aligned
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    //Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << .1, 0.0, 0.0;
    // The same rotation matrix as before; tetha radians arround Z axis
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
    // Print the transformation
    printf ("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;
    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_shelf (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*shelf, *transformed_shelf, transform_2);

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //    Run ICP
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (1000);
    icp.setInputSource(transformed_shelf);
    icp.setInputTarget(scene);
    cout<< "Running icp ... wait a moment " << endl;
    icp.align(*Final);
    cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << endl;
    cout << icp.getFinalTransformation() << endl;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //    Visualize
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (scene, "scene"); 
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5f, 0.5f, 0.5f, "scene");

    viewer->addPointCloud<pcl::PointXYZ> (transformed_shelf, "shelf"); 
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9f, 0.1f, 0.1f, "shelf");

    viewer->addPointCloud<pcl::PointXYZ> (Final, "shelf_aligned"); 
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1f, 0.9f, 0.1f, "shelf_aligned");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    //boost::share_ptr<pcl::visualization::PCLVisualization> p(new pcl::visualization::PCLVisualization("clouds")); 
    //int v1(0); 
    //int v2(0); 
    //p->createViewPort(0.0,0.0,0.5,1.0,v1);//create the first view port 
    //p->createViewPort(0.5,0.0,0.5,1.0,v2);//create the secondview port 
    //p->addPointCloud<pcl::PointXYZ>(scene,"cloud1",v1);//show clouds in the first viewport with ID is cloud1 
    //p->addPointCloud<pcl::PointXYZ>(shelf,"clouds2",v2); 

    //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //viewer.showCloud (scene);
    //viewer.addPointCloud<pcl::PointXYZ>(scene);
    //viewer.addPointCloud<pcl::PointXYZ>(shelf);
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
