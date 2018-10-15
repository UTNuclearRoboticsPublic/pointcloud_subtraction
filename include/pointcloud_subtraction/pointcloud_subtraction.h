
#ifndef POINTCLOUD_SUBTRACTION_H
#define POINTCLOUD_SUBTRACTION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

// Creates a DIFFERENCE cloud which has all the points of MINUEND, except those in common with SUBTRAHEND
// 	  That is, DIFFERENCE = MINUEND - SUBTRAHEND
// Users can either use the ROS function (subtractClouds) or the PCL-templated function (templatedSubtraction)
// The former implements the latter, but isolates the user from PCL PointType templates
namespace PointcloudSubtraction
{
// These two functions have implmentations in POINTCLOUD_SUBTRACTION.HPP
    sensor_msgs::PointCloud2 subtractClouds(sensor_msgs::PointCloud2 minuend, sensor_msgs::PointCloud2 subtrahend, bool check_rgb=true, bool check_intensity=true, bool check_normals=true, float min_dist=0.0, int num_neighbors_compared=3);
	template <typename PointType> sensor_msgs::PointCloud2 templatedSubtraction(sensor_msgs::PointCloud2 minuend, sensor_msgs::PointCloud2 subtrahend, bool check_rgb, bool check_intensity, bool check_normals, float min_dist, int num_neighbors_compared);
// This function has its implementation in COMPARE_POINTS.HPP
    template <typename PointType> bool comparePoints(PointType minuend_point, PointType subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals);
}

// Breaking implementations out into separate files for legibility
#include "pointcloud_subtraction/compare_points.hpp"
#include "pointcloud_subtraction/pointcloud_subtraction.hpp"

#endif //POINTCLOUD_SUBTRACTION_H
