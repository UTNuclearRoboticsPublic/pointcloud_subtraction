
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "pointcloud_subtraction/pointcloud_subtraction.h"

// Rosbag Stuff
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <iostream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "wall_change_tester");

    ros::NodeHandle nh;

    std::string bag_file_name, bag_file_name2;
    nh.param<std::string>("tester/bag_file", bag_file_name, "/home/conor/ros_data/Fake_Walls/Segmented/huge_flat.bag");   //"/home/conor/ros_data/Fake_Walls/Faro/Wall4.bag");
    nh.param<std::string>("tester/bag_file", bag_file_name2, "/home/conor/ros_data/Fake_Walls/Segmented/huge_flat.bag");

    ROS_INFO_STREAM("[tester] starting up, trying to load bags with names " << bag_file_name << " and " << bag_file_name2);

    // ------------------------- Load From Bags -------------------------
    rosbag::Bag cloud_bag1; 
    cloud_bag1.open(bag_file_name, rosbag::bagmode::Read);

    sensor_msgs::PointCloud2 first_cloud, second_cloud;

    std::vector<std::string> topics1;
    topics1.push_back("target_wall"); // /laser_stitcher/full_scan");
    rosbag::View view_cloud(cloud_bag1, rosbag::TopicQuery(topics1));

    BOOST_FOREACH(rosbag::MessageInstance const m, view_cloud)
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_ptr != NULL)
            first_cloud = *cloud_ptr;
        else
            ROS_ERROR_STREAM("[tester] Cloud retrieved from bag is null...");
    }
    cloud_bag1.close();
    first_cloud.header.frame_id = "map";
    first_cloud.header.stamp = ros::Time::now();


    rosbag::Bag cloud_bag2; 
    cloud_bag2.open(bag_file_name2, rosbag::bagmode::Read);

    std::vector<std::string> topics2;
    topics2.push_back("target_wall");
    rosbag::View view_cloud2(cloud_bag2, rosbag::TopicQuery(topics2));

    BOOST_FOREACH(rosbag::MessageInstance const m, view_cloud2)
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_ptr != NULL)
            second_cloud = *cloud_ptr;
        else
            ROS_ERROR_STREAM("[tester] Cloud retrieved from bag is null...");
    }
    cloud_bag2.close();
    second_cloud.header.frame_id = "map";
    second_cloud.header.stamp = ros::Time::now();
    ROS_INFO_STREAM("[tester] Loaded clouds from bag files. Sizes are: " << first_cloud.height*first_cloud.width << " and " << second_cloud.height*second_cloud.width);
    ros::Duration(0.5).sleep();
    // --------------------------------------------------------------------

    ROS_INFO_STREAM("[tester] attempting to offset faro cloud...");
    pcl::PointCloud<pcl::PointXYZI>::Ptr faro_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(first_cloud, *faro_cloud);

    // ----- Clip cloud ----- 
    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(faro_cloud);
    // Set dimensions of clipping box:
    Eigen::Vector4f min_point = Eigen::Vector4f(-5, -5, -0.1, 0);
    Eigen::Vector4f max_point = Eigen::Vector4f(5, 5, 0.3, 0);
    crop.setMin(min_point);
    crop.setMax(max_point);
    // Set pose of clipping box: 
    Eigen::Vector3f translation = Eigen::Vector3f(0.0, 0.0, 0.0);
    Eigen::Vector3f rotation = Eigen::Vector3f(0.0, 0.0, 0.0);
    crop.setTranslation(translation);
    crop.setRotation(rotation);   
    crop.setKeepOrganized(false);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_pcp(new pcl::PointCloud<pcl::PointXYZI>());
    crop.filter(*temp_pcp);
    *faro_cloud = *temp_pcp;

    sensor_msgs::PointCloud2 clipped_msg;
    toROSMsg(*temp_pcp, clipped_msg);
    sensor_msgs::PointCloud2 difference_cloud;



    difference_cloud = PointcloudSubtraction::subtractClouds(first_cloud, clipped_msg, true, true, true, 0.001, 3);
    ros::Publisher minuend_pub = nh.advertise<sensor_msgs::PointCloud2>("minuend", 1);
    ros::Publisher subtrahend_pub = nh.advertise<sensor_msgs::PointCloud2>("subtrahend", 1);
    ros::Publisher difference_pub = nh.advertise<sensor_msgs::PointCloud2>("difference", 1);
    while(ros::ok())
    {
        minuend_pub.publish(first_cloud);
        subtrahend_pub.publish(clipped_msg);
        difference_pub.publish(difference_cloud);    
        ros::Duration(1.0).sleep();
    }
}