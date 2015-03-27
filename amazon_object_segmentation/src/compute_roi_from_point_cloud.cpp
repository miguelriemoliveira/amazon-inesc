#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <rospack/rospack.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/server/simple_action_server.h>
//#include <amazon_msgs/ComputeROIFromPointCloudAction.h>
#include <amazon_msgs/Point2DInteger.h>

#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>

#include <tf/transform_listener.h>

using namespace std;
using namespace ros;
using namespace ros::package;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class ActionServer
{
    protected:

        tf::TransformListener listener;
        ros::NodeHandle nh_; // NodeHandle instance must be created before this line. Otherwise strange error may occur.

        actionlib::SimpleActionServer<amazon_msgs::ComputeROIFromPointCloudAction> as_; 
        std::string action_name_;

        // create messages that are used to published feedback/result
        amazon_msgs::ComputeROIFromPointCloudFeedback feedback_;
        amazon_msgs::ComputeROIFromPointCloudResult result_;

    public:

        ActionServer(std::string name): as_(nh_, name, boost::bind(&ActionServer::executeCB, this, _1), false), action_name_(name)
    {
        as_.start();
    }

        ~ActionServer(void)
        {
        }

        void executeCB(const amazon_msgs::ComputeROIFromPointCloudGoalConstPtr &goal)
        {
            bool success = true;
            ROS_INFO_STREAM("Received action goal for segmenting the point cloud");
            //TODO should set the goal to active. Read more about the actionlib state machine

            if(!as_.isActive()||as_.isPreemptRequested())
            {
                ROS_INFO_STREAM("Received action goal for segmenting the point cloud");
                success = false;
                as_.setAborted(result_);
                return;
            }



            //pcl::PointCloud<PointT>::Ptr pc;
            //pc = (pcl::PointCloud<PointT>::Ptr) new (pcl::PointCloud<PointT>);


            //sensor_msgs::PointCloud2 pcmsg_transformed;

            //if (listener.waitForTransform("/shelf", goal->point_cloud.header.frame_id, goal->point_cloud.header.stamp, ros::Duration(0.1)))
            //{
                //pcl_ros::transformPointCloud("shelf", goal->point_cloud, pcmsg_transformed, listener);
            //}
            //else
            //{
                //success = false;
                //result_.result = 0; 
                //as_.setAborted(result_);
            //}

            ////convert from ros msg to pcl (already removes RGB component because IN is pcl::PointXYZ)
            //pcl::fromROSMsg(pcmsg_transformed, *pc);

            ////Define a pick_region
            //pcl::PointCloud<pcl::PointXYZ>::Ptr pick_region (new pcl::PointCloud<pcl::PointXYZ> ());
            //float security_x = 0.00;
            //float security_y = 0.01;
            //float dx = bin_pose_->at(goal->bin_number*2);
            //float dy = bin_pose_->at(goal->bin_number*2+1);
            //float prx = bin_size_->at(0);
            //float pry = bin_size_->at(1);

            //pcl::PointXYZ p1,p2,p3,p4,p5;
            //p1.x = dx + security_x; p1.y = dy + security_y; p1.z = 0;
            //pick_region->points.push_back(p1);

            //p2.x = dx+prx - security_x; p2.y = dy + security_y; p2.z = 0;
            //pick_region->points.push_back(p2);

            //p3.x = dx+prx  - security_x; p3.y = dy+pry - security_y; p3.z = 0;
            //pick_region->points.push_back(p3);

            //p4.x = dx + security_x; p4.y = dy+pry - security_y; p4.z = 0;
            //pick_region->points.push_back(p4);

            //p5.x = dx + security_x; p5.y = dy + security_y; p5.z = 0;
            //pick_region->points.push_back(p5);

            //pcl::ModelCoefficients::Ptr coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());

            ////coefficients->values
            //coefficients->values.push_back(0.0) ;
            //coefficients->values.push_back(0.0) ;
            //coefficients->values.push_back(1.0) ;
            //coefficients->values.push_back(0.0) ;

            //typedef pcl::PointXYZ T;
            //pcl::ExtractPolygonalPrismData<PointT> epp;
            //pcl::PointIndices::Ptr indices = pcl::PointIndices::Ptr(new pcl::PointIndices);

            //////Set epp parameters
            //epp.setInputCloud(pc);
            //pcl::PointCloud<PointT>::Ptr pick_regionRGB (new pcl::PointCloud<PointT> ());
            //pcl::copyPointCloud(*pick_region, *pick_regionRGB);

            //epp.setInputPlanarHull(pick_regionRGB);
            //float shelf_depth = bin_size_->at(2);
            //epp.setHeightLimits(-0.1,shelf_depth); 
            //epp.setViewPoint(0,0,0); //i dont think this serves any purpose in the case of epp
            //epp.segment(*indices);

            //pcl::PointCloud<PointT>::Ptr points_in_binA (new pcl::PointCloud<PointT> ());
            //pcl::ExtractIndices<PointT> extract1; //Create the extraction object
            //extract1.setInputCloud(pc);
            //extract1.setIndices(indices);
            //extract1.setNegative(false);
            //extract1.filter(*points_in_binA);


            //pcl::toROSMsg(*points_in_binA, result_.segmented_point_cloud);

            //ROS_INFO("Extracted %ld points from bin %s. Original pc with %ld points.",points_in_binA->points.size(), (bin_label_->at(goal->bin_number)).c_str(), pc->points.size());
            ///[>pick_regionRGB = *pick_region;
            //// push_back the seeds for the fibonacci sequence
            ////feedback_.feedback = "action ongoing";

            //// publish info to the console for the user
            ////ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

            ////start executing the action
            //// check that preempt has not been requested by the client
            //if (as_.isPreemptRequested() || !ros::ok())
            //{
                //ROS_INFO("%s: Preempted", action_name_.c_str());
                //// set the action state to preempted
                //as_.setPreempted();
                //success = false;
            //}
            ////feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);

            //// publish the feedback
            ////as_.publishFeedback(feedback_);


            //if(success)
            //{
                //result_.result = 1;
                //ROS_INFO("%s: Succeeded", action_name_.c_str());
                //// set the action state to succeeded
                //as_.setSucceeded(result_);
            //}
            //else
            //{
                //as_.setAborted(result_);
            //}
        }


};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "segment_point_cloud_from_bin");
    ros::NodeHandle nh;

    ros::Duration(0.1).sleep(); // sleep for half a second

    ActionServer action_server(ros::this_node::getName());

    ros::Rate loop_rate(20);

    ros::spin();
    //while (ros::ok())
    //{

        //ros::spinOnce();
        //loop_rate.sleep();

        //for (size_t i=0; i < ma.markers.size(); i++)
            //ma.markers[i].header.stamp = ros::Time::now();
        //marker_pub.publish(ma);
    //}



    return (0);
}
