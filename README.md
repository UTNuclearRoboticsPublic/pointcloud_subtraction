# Table of Contents
1. [About](#about)
2. [Usage](#usage)

## About
This package is to troll through one pointcloud and check each of its points against those in another cloud, removing all points which have a close match in the second cloud. The match can be forced to be exact in position, color, intensity, or normal data, or it can be based on some specified threshold.

For the sake of the rest of this document, the following terms are used to describe the operands of subtraction:

difference = minuend - subtrahend

This is NOT the preferred method to remove the contents of one cloud from another. Because a KDTree search has to be performed for every point in the minuend cloud against the points in the subtrahend cloud, this algorithm can get inefficient for very large clouds. The complexity is N log(n), where N is the size of the minuend cloud and n is the size of the subtrahend. 

In a case where points are selected from within a cloud to be removed, it is preferable to just save the indices of the points within the larger cloud during the original process which created the list of points, negating the need for a later search (as here). This is how most PCL functions work. The kind of search implemented here is only necessary when the two clouds have different sensor origins, or if the indicies of matching points don't align between the clouds and no saved version of the original indices was made. 

All that said, this kind of utility can be fairly powerful for purposes such as to remove all the points in a newly generated cloud which match within some threshold of the points in a particular object segmented out of a previous scene. EG, if we've found the walls in the past, we could cut them out of future scenes without segmenting again. Whether this is useful computationally or semantically will obviously vary with case. 

## Usage
This package doesn't include any executables, only a library to be used in other packages. Make sure to find_package this package (pointcloud_subtraction) in the CMakeLists.txt for the client package, and #include <pointcloud_subtraction/pointcloud_subtraction.h>

The syntax to use the main subtraction function in cpp is as follows:

```
sensor_msgs::PointCloud2 output_msg = PointcloudSubtraction::subtractClouds(minuend, subtrahend, check_rgb, check_intensity, check_normals, min_dist, num_neighbors_compared);
```

- **minuend/subtrahend** *(sensor_msgs::PointCloud2):*   the operands of subtraction;  difference = minuend - subtrahend
- **check_rgb/intensity/normals** *(bool):*   whether points must match in color, return intensity, and normal data in order to be considered identical and subtracted
- **min_dist** *(float):*   the threshold at which points are considered close enough to be subtracted
- **num_neighbors_compared** *(int):*   the number of neighbors to be checked against. Logic should dictate that this should be only one, to find the single nearest point that should be removed, except that breaks down in cases where multiple points are coincident. I had the best results with 3 here but a decision will have to be made based on the nature of the data. 
