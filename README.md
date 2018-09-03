# Table of Contents
1. [About](#about)
2. [Usage](#usage)

## About
This package is to troll through one pointcloud and check each of its points against those in another cloud, removing all points which have a close match in the second cloud. The match can be forced to be exact in position, color, intensity, or normal data, or it can be based on some specified threshold.

This is NOT the preferred method to remove the contents of one cloud from another. Because a KDTree search has to be performed for every point minuend cloud against the points in the subtrahend cloud, this algorithm can get inefficient for very large clouds. The complexity is N log(n), where N is the size of the minuend cloud and n is the size of the subtrahend. 

In a case where points are selected from within a cloud to be removed, it is preferable to just save the indices of the points within the larger cloud during the selection process. This is how most PCL funcitons work. The kind of search implemented here is only necessary when the two clouds have different origins, or if the indicies of matching points don't align between the clouds and no saved version of the original indices was made. 

## Usage
This package doesn't include any executables, only a library to be used in other packages. Make sure to find_package this package (pointcloud_subtraction) in the CMakeLists.txt for the client package, and #include <pointcloud_subtraction/pointcloud_subtraction.h>

Usage works as follows:

sensor_msgs::PointCloud2 output_msg = PointcloudSubtraction::subtractClouds(minuend, subtrahend, check_rgb, check_intensity, check_normals, min_dist, num_neighbors_compared);

- minuend/subtrahend: these are both sensor_msgs::PointCloud2 messages; the logic is  output_msg = minuend - subtrahend
- check_rgb/intensity/normals: whether points must match in color, return intensity, and normal data in order to be considered identical and subtracted
- min_dist: the threshold at which points are considered close enough to be subtracted
- num_neighbors_compared: the number of neighbors to be checked against. Logic should dictate that this should be only one, to find the single nearest point that should be removed, except that breaks down in cases where multiple points are coincident. I had the best results with 3 here but a decision will have to be made based on the nature of the data. 
