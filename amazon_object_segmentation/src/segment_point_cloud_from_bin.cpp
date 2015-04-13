#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <rospack/rospack.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/server/simple_action_server.h>
#include <amazon_msgs/SegmentBinAction.h>

#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>

#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

using namespace std;
using namespace ros;
using namespace ros::package;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class SegmentBinAction
{
    protected:

        tf::TransformListener listener;
        ros::NodeHandle nh_; // NodeHandle instance must be created before this line. Otherwise strange error may occur.

        actionlib::SimpleActionServer<amazon_msgs::SegmentBinAction> as_; 
        std::string action_name_;

        // create messages that are used to published feedback/result
        amazon_msgs::SegmentBinFeedback feedback_;
        amazon_msgs::SegmentBinResult result_;

        boost::shared_ptr <vector<string> > bin_label_;
        boost::shared_ptr <vector<double> > bin_size_;
        boost::shared_ptr <vector<double> > bin_pose_;

    public:

        SegmentBinAction(std::string name, boost::shared_ptr <vector<string> > bin_label, boost::shared_ptr <vector<double> > bin_size,  boost::shared_ptr <vector<double> > bin_pose): as_(nh_, name, boost::bind(&SegmentBinAction::executeCB, this, _1), false), action_name_(name)
    {
        bin_label_ = bin_label;
        bin_size_ = bin_size;
        bin_pose_ = bin_pose;
        as_.start();
    }

        ~SegmentBinAction(void)
        {
        }

        void executeCB(const amazon_msgs::SegmentBinGoalConstPtr &goal)
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


            ROS_INFO("Waiting for camera_info message on topic %s",goal->camera_info_topic.c_str());
            double time_to_wait_for_message = 1.0;
            sensor_msgs::CameraInfo::ConstPtr camera_info_msg = topic::waitForMessage<sensor_msgs::CameraInfo>(goal->camera_info_topic, nh_, ros::Duration(time_to_wait_for_message));

            if (!camera_info_msg)
            {
                ROS_ERROR("No camera_info message has been received after %f secs", time_to_wait_for_message);
                as_.setAborted(result_);
                return;
            }
            else
            {
                ROS_INFO_STREAM("Received camera_info message");
            }

            image_geometry::PinholeCameraModel _cam_model;
            _cam_model.fromCameraInfo(camera_info_msg);


            pcl::PointCloud<PointT>::Ptr pc;
            pc = (pcl::PointCloud<PointT>::Ptr) new (pcl::PointCloud<PointT>);
            sensor_msgs::PointCloud2 pcmsg_transformed;

            if (listener.waitForTransform("/shelf", goal->point_cloud.header.frame_id, goal->point_cloud.header.stamp, ros::Duration(0.1)))
            {
                pcl_ros::transformPointCloud("/shelf", goal->point_cloud, pcmsg_transformed, listener);
            }
            else
            {
                success = false;
                as_.setAborted(result_);
            }

            //convert from ros msg to pcl (already removes RGB component because IN is pcl::PointXYZ)
            pcl::fromROSMsg(pcmsg_transformed, *pc);

            ROS_INFO("goal->point_cloud %s", goal->point_cloud.header.frame_id.c_str());
            ROS_INFO("pc frame_id %s", pc->header.frame_id.c_str());

            //Define a pick_region
            pcl::PointCloud<pcl::PointXYZ>::Ptr pick_region (new pcl::PointCloud<pcl::PointXYZ> ());
            pick_region->header.frame_id = "/shelf";
            float security_x = 0.00;
            float security_y = 0.01;
            float dx = bin_pose_->at(goal->bin_number*2);
            float dy = bin_pose_->at(goal->bin_number*2+1);
            float prx = bin_size_->at(0);
            float pry = bin_size_->at(1);

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

            pcl::ModelCoefficients::Ptr coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());

            //coefficients->values
            coefficients->values.push_back(0.0) ;
            coefficients->values.push_back(0.0) ;
            coefficients->values.push_back(1.0) ;
            coefficients->values.push_back(0.0) ;

            typedef pcl::PointXYZ T;
            pcl::ExtractPolygonalPrismData<PointT> epp;
            pcl::PointIndices::Ptr indices = pcl::PointIndices::Ptr(new pcl::PointIndices);

            ////Set epp parameters
            epp.setInputCloud(pc);
            pcl::PointCloud<PointT>::Ptr pick_regionRGB (new pcl::PointCloud<PointT> ());
            pcl::copyPointCloud(*pick_region, *pick_regionRGB);

            epp.setInputPlanarHull(pick_regionRGB);
            float shelf_depth = bin_size_->at(2);
            epp.setHeightLimits(-0.1,shelf_depth); 
            epp.setViewPoint(0,0,0); //i dont think this serves any purpose in the case of epp
            epp.segment(*indices);

            pcl::PointCloud<PointT>::Ptr points_in_binA (new pcl::PointCloud<PointT> ());
            pcl::PointCloud<PointT>::Ptr points_in_binA_transformed (new pcl::PointCloud<PointT> ());
            pcl::ExtractIndices<PointT> extract1; //Create the extraction object
            extract1.setInputCloud(pc);
            extract1.setIndices(indices);
            extract1.setNegative(false);
            extract1.filter(*points_in_binA);

            if (listener.waitForTransform(goal->point_cloud.header.frame_id, "/shelf", goal->point_cloud.header.stamp, ros::Duration(0.1)))
            {
                pcl_ros::transformPointCloud(goal->point_cloud.header.frame_id, *points_in_binA, *points_in_binA_transformed, listener);
            }
            else
            {
                success = false;
                as_.setAborted(result_);
            }

            ROS_INFO("points_in_binA frame_id %s", points_in_binA->header.frame_id.c_str());
            ROS_INFO("points_in_binA_transformed frame_id %s", points_in_binA_transformed->header.frame_id.c_str());

            //Project to image
            vector<cv::Point2i> lpixels;

            for (size_t i=0; i < points_in_binA_transformed->points.size(); i++ )
            {
                cv::Point3d pt_cv(points_in_binA_transformed->points[i].x, points_in_binA_transformed->points[i].y, points_in_binA_transformed->points[i].z);
                lpixels.push_back(_cam_model.project3dToPixel(pt_cv));
            }

            cv::Mat mask_img(camera_info_msg->width, camera_info_msg->height, cv::DataType<bool>::type);
            cv::Rect roi = cv::boundingRect(lpixels); 
            result_.roi.x_offset = roi.x;
            result_.roi.y_offset = roi.y;
            result_.roi.width = roi.width;
            result_.roi.height = roi.height;


            //vector<cv::Point> hull;
            //cv::convexHull( mask_img, hull, false );

            pcl::toROSMsg(*points_in_binA_transformed, result_.segmented_point_cloud);

            ROS_INFO("Extracted %ld points from bin %s. Original pc with %ld points.",points_in_binA->points.size(), (bin_label_->at(goal->bin_number)).c_str(), pc->points.size());

            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
            }

            if(success)
            {
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                // set the action state to succeeded
                as_.setSucceeded(result_);
            }
            else
            {
                as_.setAborted(result_);
            }
        }


};



visualization_msgs::Marker create_bin_label(string label, string ns, string frame_id, double x, double y)
{
    static int id = 0 ; 
    visualization_msgs::Marker m;
    m.ns = ns;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    //m.pose.orientation.w = 1.0;
    m.id = id++;
    //m.lifetime = 0;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.scale.z = 0.1;
    m.text = label;

    m.pose.position.x = x;
    m.pose.position.y = y;
    // Color
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = 1.0;

    return m;
}

visualization_msgs::Marker create_bin_cube(string ns, string frame_id, double x, double y, vector<double>& size)
{
    static int id = 0 ; 
    geometry_msgs::Point p;
    visualization_msgs::Marker m;

    m.ns = ns;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.id = id++;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.scale.x = .005;

    // Color
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = 1.0;

    p.x = x; p.y = y; p.z = 0;
    m.points.push_back(p);
    p.x += size[0]; 
    m.points.push_back(p);
    p.y += size[1]; 
    m.points.push_back(p);
    p.x -= size[0]; 
    m.points.push_back(p);
    p.y -= size[1]; 
    m.points.push_back(p);
    p.z += size[2];
    m.points.push_back(p);
    p.x += size[0]; 
    m.points.push_back(p);
    p.y += size[1]; 
    m.points.push_back(p);
    p.x -= size[0]; 
    m.points.push_back(p);
    p.y -= size[1]; 
    m.points.push_back(p);
    p.y += size[1]; 
    m.points.push_back(p);
    p.z -= size[2]; 
    m.points.push_back(p);
    p.x += size[0]; 
    m.points.push_back(p);
    p.z += size[2]; 
    m.points.push_back(p);
    p.y -= size[1]; 
    m.points.push_back(p);
    p.z -= size[2]; 
    m.points.push_back(p);

    return m;
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "segment_point_cloud_from_bin");
    ros::NodeHandle nh;

    //Problem linking? check http://answers.ros.org/question/196935/roslib-reference-error/
    string path = ros::package::getPath("amazon_object_segmentation");
    ros::Duration(0.1).sleep(); // sleep for half a second



    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("shelf_and_bins_marker", 10);

    visualization_msgs::MarkerArray ma;

    //get the params of the shelf
    vector<string> bin_label;
    nh.getParam("/shelf/BinLabel",bin_label);

    vector<double> bin_size;
    nh.getParam("/shelf/BinSize",bin_size);

    vector<double> bin_pose;
    nh.getParam("/shelf/BinPose",bin_pose);


    for (size_t i=0; i<bin_label.size(); i++)
    {
        visualization_msgs::Marker m = create_bin_label(bin_label[i],"labels","shelf", bin_pose[2*i] + bin_size[0]/2,bin_pose[1 + 2*i] + bin_size[1]);
        ma.markers.push_back(m);
        ma.markers.push_back(create_bin_cube("binA","shelf", bin_pose[2*i], bin_pose[1 + 2*i], bin_size));
    }

    SegmentBinAction segment_bin_action(ros::this_node::getName(), boost::make_shared<vector<string> >(bin_label), boost::make_shared<vector<double> >(bin_size), boost::make_shared<vector<double> >(bin_pose));

    ros::Rate loop_rate(20);

    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();

        for (size_t i=0; i < ma.markers.size(); i++)
            ma.markers[i].header.stamp = ros::Time::now();
        marker_pub.publish(ma);
    }



    return (0);
}
