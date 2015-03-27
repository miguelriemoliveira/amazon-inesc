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
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>


#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__); 
using namespace std;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);

//// Executing the transformation
//pcl::PointCloud<PointT>::Ptr transformed_scene (new pcl::PointCloud<PointT> ());
//pcl::transformPointCloud (*scene, *transformed_scene, tinv);


////Work on the transformed point cloud


int main (int argc, char** argv)
{
    ros::init(argc, argv, "test_segment");

    scene = (pcl::PointCloud<PointT>::Ptr) new (pcl::PointCloud<PointT>);
    vector<int> indicesNAN;
    float theta = M_PI/20; // The angle of rotation in radians


    //string path = ros::package::getPath("amazon_object_segmentation");
    //Load and remove NANs
    if (pcl::io::loadPCDFile<PointT> ("scene_calibrated.pcd", *scene) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    pcl::removeNaNFromPointCloud(*scene, *scene, indicesNAN);
    cout << "scene pc has " << scene->points.size() << " points" << endl;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //    Visualize
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(scene);
    viewer->addPointCloud<PointT> (scene, rgb, "scene"); 
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();

    //1.48244,5.11656/0.402184,0.108397,-0.0249941/0.439225,-0.863033,-2.41882/-0.00581954,-0.926626,0.37594/0.523599/908,763/145,162
    ///0.402184,0.108397,-0.0249941/0.439225,-0.863033,-2.41882/-0.00581954,-0.926626,0.37594/0.523599/908,763/145,162


    //viewer->setBackgroundColor(0, 0, 0); 
    viewer->setBackgroundColor(1, 1, 1); 
    viewer->setCameraPosition(0.0497412, -0.196849, -0.0978747, 0.0427789, -0.185814, 0.0496169, -0.0956887, -0.992963, 0.0697719); 
    viewer->setCameraFieldOfView(0.523599); 
    viewer->setCameraClipDistances(1.48244,5.11656); 
    viewer->setPosition(1650, 152); 
    viewer->setSize(631, 491); viewer->updateCamera(); 
    //Add viewer callbacks:


    //boost::share_ptr<pcl::visualization::PCLVisualization> p(new pcl::visualization::PCLVisualization("clouds")); 
    //int v1(0); 
    //int v2(0); 
    //p->createViewPort(0.0,0.0,0.5,1.0,v1);//create the first view port 
    //p->createViewPort(0.5,0.0,0.5,1.0,v2);//create the secondview port 
    //p->addPointCloud<PointT>(scene,"cloud1",v1);//show clouds in the first viewport with ID is cloud1 
    //p->addPointCloud<PointT>(scene,"clouds2",v2); 

    //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //viewer.showCloud (scene);
    //viewer.addPointCloud<PointT>(scene);
    //viewer.addPointCloud<PointT>(scene);
    //while (!viewer.wasStopped ())
    //{
    //}


    //pcl::PointXYZ pta(0,0,0);
    //pcl::PointXYZ ptb(10,10,10);
    //viewer->addLine<pcl::PointXYZ> (pta, ptb, 1.0 , 0.0, 0.0, "sads");

    //Define a pick_region
    pcl::PointCloud<pcl::PointXYZ>::Ptr pick_region (new pcl::PointCloud<pcl::PointXYZ> ());
    float security_x = 0.00;
    float security_y = 0.01;
    float dx = 0.034 ;
    float dy = 0.032;
    float prx = 0.276;
    float pry = 0.24;

    pcl::PointXYZ p1,p2,p3,p4,p5;
    p1.x = dx + security_x; p1.y = dy + security_y; p1.z = 0;
    pick_region->points.push_back(p1);

    p2.x = dx+prx - security_x; p2.y = dy + security_y; p2.z = 0;
    pick_region->points.push_back(p2);

    p3.x = dx+prx  - security_x; p3.y = dy+pry - security_y; p3.z = 0;
    pick_region->points.push_back(p3);

    p4.x = dx + security_x; p4.y = dy+pry - security_y; p4.z = 0;
    pick_region->points.push_back(p4);

    p5.x = dx + security_x; p5.y = dy + security_y; p5.z = 0;
    pick_region->points.push_back(p5);

    cout << "size = " << pick_region->points.size() << endl;

    PFLN
    for (size_t i=0; i<pick_region->points.size()-1; i++)
    {
        string s = "line" + i;
        viewer->addLine<pcl::PointXYZ> (pick_region->points[i], pick_region->points[i+1], 1.0 , 0.0, 0.0, s);
        //viewer->addLine<pcl::PointXYZ> (pt1, pt2, s);

    }

    PFLN
    pcl::ModelCoefficients::Ptr coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());

    //pcl::ModelCoefficients::Ptr coefficients;
    //coefficients = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;
    PFLN
    //couefficients->values
    coefficients->values.push_back(0.0) ;
    coefficients->values.push_back(0.0) ;
    coefficients->values.push_back(1.0) ;
    coefficients->values.push_back(0.0) ;
    
    typedef pcl::PointXYZ T;
    pcl::ExtractPolygonalPrismData<PointT> epp;                                
	pcl::PointIndices::Ptr indices = pcl::PointIndices::Ptr(new pcl::PointIndices);

    //polygon->points.push_back(polygon->points[0]); //because the pcl ch polygon needs the first and last points to be the same
	////Set epp parameters
	epp.setInputCloud(scene);


    pcl::PointCloud<PointT>::Ptr pick_regionRGB (new pcl::PointCloud<PointT> ());
    pcl::copyPointCloud(*pick_region, *pick_regionRGB);

    //*pick_regionRGB = *pick_region;
	epp.setInputPlanarHull(pick_regionRGB);
    float shelf_depth = 0.4;
	epp.setHeightLimits(-0.1,shelf_depth); 
	epp.setViewPoint(0,0,0); //i dont think this serves any purpose in the case of epp
	epp.segment(*indices);

    pcl::PointCloud<PointT>::Ptr points_in_binA (new pcl::PointCloud<PointT> ());
	pcl::ExtractIndices<PointT> extract1; //Create the extraction object
	extract1.setInputCloud(scene);
	extract1.setIndices(indices);
	extract1.setNegative(false);
	extract1.filter(*points_in_binA);


    viewer->addPointCloud<pcl::PointXYZRGB> (points_in_binA, "points_in_binA"); 
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.7f, 0.1f, "points_in_binA");

    string file = "points_in_binA.pcd";
    cout << "Calibrated point cloud saved to " << file << endl;
    pcl::io::savePCDFileASCII (file, *points_in_binA);


    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
}
