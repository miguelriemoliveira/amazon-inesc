#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__); 
using namespace std;
using namespace pcl;
typedef PointXYZRGB PointT;
typedef PointCloud<PointT> PointCloudT;


PointCloud<PointT>::Ptr scene (new PointCloud<PointT>);

int main (int argc, char** argv)
{
    //ros::init(argc, argv, "test_segment");

    PCDWriter writer;
    scene = (PointCloud<PointT>::Ptr) new (PointCloud<PointT>);
    vector<int> indicesNAN;
    float theta = M_PI/20; // The angle of rotation in radians

    if (io::loadPCDFile<PointT> ("points_in_binA.pcd", *scene) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    removeNaNFromPointCloud(*scene, *scene, indicesNAN);
    cout << "scene pc has " << scene->points.size() << " points" << endl;


    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>);
    tree->setInputCloud (scene);

    vector<PointIndices> cluster_indices;
    EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (scene);
    ec.extract (cluster_indices);

    boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1); 
    viewer->setCameraPosition(0.0497412, -0.196849, -0.0978747, 0.0427789, -0.185814, 0.0496169, -0.0956887, -0.992963, 0.0697719); 
    viewer->setCameraFieldOfView(0.523599); 
    viewer->setCameraClipDistances(1.48244,5.11656); 
    viewer->setPosition(1650, 152); 
    viewer->setSize(631, 491); viewer->updateCamera(); 

    int j = 0;
    for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PointCloud<PointT>::Ptr cloud_cluster (new PointCloud<PointT>);
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (scene->points[*pit]); 
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        string name = "cluster" + j;
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud_cluster, name); 
        
        if (j==0)
        {
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.7f, 0.1f, name);
        }
        else if (j==1)
        {
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.7f, 1.0f, name);
        }
        else if (j==2)
        {
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, name);
        }



        cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
        stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<PointT> (ss.str (), *cloud_cluster, false); 
        j++;
    }

    
   
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


    return (0);
}
