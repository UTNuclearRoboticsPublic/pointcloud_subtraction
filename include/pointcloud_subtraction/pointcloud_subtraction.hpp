
namespace PointcloudSubtraction
{
	// ----------------------------------------------------------------------------------------------------------------------
	// *** TEMPLATED SUBTRACTION ***
	//   Performs actual search through MINUEND for points that are common with SUBTRAHEND
	//   Creates a new cloud DIFFERENCE which contains all MINUEND points except those found above
	//   This function is templated across PCL point types
	//   Currently, the following types are supported:
	//     PointXYZ, PointXYZI, PointNormal, PointXYZINormal, PointRGB, PointRGBNormal
	//   That is - points containing XYZ data plus any combination of I, Normal, and RGB data 
    template <typename PointType> sensor_msgs::PointCloud2 templatedSubtraction(sensor_msgs::PointCloud2 minuend, sensor_msgs::PointCloud2 subtrahend, bool check_rgb, bool check_intensity, bool check_normals, float min_dist, int num_neighbors_compared)
    {
    	// -------------------------- INITIAL SETUP ----------------------------

    	typedef typename pcl::PointCloud<PointType>             PC;
    	typedef typename pcl::PointCloud<PointType>::Ptr        PCP;
    	typedef typename pcl::KdTreeFLANN<PointType>            KDTree; 

    	// PCL cloud types
        PCP minuend_ptr(new PC());
        PCP subtrahend_ptr(new PC());
        PCP difference_ptr(new PC());

        // Convert input from ROS message type to PCL
        pcl::fromROSMsg(minuend, *minuend_ptr);
        pcl::fromROSMsg(subtrahend, *subtrahend_ptr);

        // Search structure for NEAREST NEIGHBOR search (in XYZ-space)
        KDTree kdtree;
        kdtree.setInputCloud(minuend_ptr);
        std::vector<float> nearest_dist_squareds;
        std::vector<int> nearest_indices;

        // --------------------------- SEARCH FOR COMMON POINTS ---------------------------

        // List of INDICES from MINUEND that match points in SUBTRAHEND - those to be subtracted
        pcl::PointIndices::Ptr indices_to_subtract(new pcl::PointIndices);
        // Also create a set concurrently, because it's a lot more efficient to check whether an index is a duplicate within a set (Ologn) than a vector (On), since sets are ordered
        std::set<int> index_set;
        ROS_DEBUG_STREAM("[PointcloudSubtraction] Created PCL clouds, about to perform KD Search for neighbors across clouds...");
        // ITERATE over SUBTRAHEND points
        for(int i=0; i<subtrahend_ptr->points.size(); i++)
        	// SEARCH for NEIGHBORS of a subtrahend point in minuend
            if ( kdtree.nearestKSearch (subtrahend_ptr->points[i], num_neighbors_compared, nearest_indices, nearest_dist_squareds) > 0 )
            	// ITERATE over NEIGHBOR list
            	for (int j=0; j<nearest_indices.size(); j++)
            		// COMPARE XYZ values between points 
                	if ( nearest_dist_squareds[j] <= min_dist )  
                		// COMPARE RGB, INTENSITY, NORMALS values between points (the below function is templated on the same parameters as this function)
                    	if (comparePoints(minuend_ptr->points[nearest_indices[j]], subtrahend_ptr->points[i], check_rgb, check_intensity, check_normals) )
                    		// CHECK if the new index is a REPEAT - this can occur if multiple points are within min_dist of the subtrahend point
                        	//    A major reason for exposing num_neighbors_compared to user input is the possibility that multiple subtrahend points identify 
                        	//     the same minuend point as their 'closest' neighbor if there are multiple points the same distance away (or even coincident).
                        	//     This can lead to the same index being repeated in the vector. This is a problem because it can lead to the OTHER point not 
                        	//     getting included and not being subtracted as a result.
                        	//    HOWEVER, this is also an issue because pcl::ExtractIndices seemingly is written to return a zero-point cloud if the list of indices
                        	//     passed is longer than the input cloud size. This is dumb, because the function actually DOES work properly when passed an index
                        	//     list containing repeat indices, as long as that list isn't LONGER than the total input cloud size, leading to unintuitive results.
                        	//     The set and uniqueness check are added below to prevent this from happening for cases where there is a LOT of nearness between 
                        	//     points (or equivalently, where a large value of min_dist is selected).
                        	if( index_set.find(nearest_indices[j]) == index_set.end() ) 	
                        	{
                        		// Add the new index to the VECTOR (actually used by the later extraction call) and the SET (used to check for duplicates efficiently)
                        		indices_to_subtract->indices.push_back(nearest_indices[j]);
                        		index_set.insert(nearest_indices[j]);
                        	}

    	ROS_DEBUG_STREAM("[PointcloudSubtraction] Neighbor search performed! " << indices_to_subtract->indices.size() << " common points were found between the clouds.");

    	// ------------------------- CREATE DIFFERENCE CLOUD -----------------------------

    	// Subtract the matching indices
    	pcl::ExtractIndices<PointType> extract (true); 	
  		extract.setInputCloud(minuend_ptr);
  		extract.setIndices(indices_to_subtract);
  		extract.setNegative(true); 		// Remove points NOT given by indices
  		// Actually segment out plane pointcloud, and set input to match new, smaller cloud
  		extract.filter(*difference_ptr); 

  		ROS_DEBUG_STREAM("[PointcloudSubtraction] Subtracted two pointclouds." << 
  				 " Minuend cloud size: " << minuend_ptr->points.size() << 
  				"; Subtrahend cloud size: " << subtrahend_ptr->points.size() << 
  				"; Difference cloud size: " << difference_ptr->points.size());
  		sensor_msgs::PointCloud2 difference;
  		pcl::toROSMsg(*difference_ptr, difference);
  		return difference;

    }	// templatedSubtraction<>()
    // ----------------------------------------------------------------------------------------------------------------------

	
	// ----------------------------------------------------------------------------------------------------------------------
    // *** SubtractClouds ***
    //   This is the outwards-facing function intended to be used by ROS users
    //   The inputs and outputs are both sensor_msgs clouds; lower-level code is written in PCL, which is templated on point type
    //   However, all sensor_msgs clouds are the same type, and encode their point type information in the FIELDS parameter (a list of the field included)
    //   If a user is working with PCL clouds instead of ROS Msgs, they might consider directly using the templatedSubtraction<> templates above
    sensor_msgs::PointCloud2 subtractClouds(sensor_msgs::PointCloud2 minuend, sensor_msgs::PointCloud2 subtrahend, bool check_rgb, bool check_intensity, bool check_normals, float min_dist, int num_neighbors_compared)
    { 
    	ROS_DEBUG_STREAM("[PointcloudSubtraction] Received call to subtract clouds. Minuend size: " << minuend.height*minuend.width << " Subtrahend size: " << subtrahend.height*subtrahend.width << " Input settings - CHECK_RGB: " << check_rgb+0 << " CHECK_INTENSITY: " << check_intensity+0 << " CHECK_NORMALS: " << check_normals+0);
        // If clouds have different numbers of fields, return the input cloud
        if(minuend.fields.size() != subtrahend.fields.size())
            ROS_WARN_STREAM("[PointcloudSubtraction] Cloud subtraction requested, but minuend and subtrahend have a different number of fields, at " << minuend.fields.size() << " and " << subtrahend.fields.size() << ", respectively. Continuing, but results might be unexpected...");

    	// ------------------------- PARSE CLOUD FIELDS -----------------------------

        // ------------ MINUEND ------------
        bool minuend_rgb = false;           // Does the minuend cloud contain RGB data??
        bool minuend_intensity = false;     // Does the minuend cloud contain INTENSITY data? 
        bool minuend_normals = false;       // Does the minuend cloud contain NORMALS data? 
        for(int i=0; i<minuend.fields.size(); i++)
        {
        	std::string name = minuend.fields[i].name;
            if(name.compare("rgb") == 0)
                minuend_rgb = true;
            else if(name.compare("intensity") == 0)
                minuend_intensity = true;
            else if(name.compare("normal_x") == 0)        // although clouds with normal data actually devote three fields to it (normal_x, normal_y, normal_z, and curvature)...
                minuend_normals = true;
            else if(name.compare("normal_y") != 0 && name.compare("normal_z") != 0 && name.compare("curvature") != 0) 	// Field is not a Normal field
        		if(name.compare("x") != 0 && name.compare("y") != 0 && name.compare("z") != 0)							// Field is not an XYZ field
                    ROS_WARN_STREAM("[PointcloudSubtraction] During cloud subtraction, minuend cloud contains an unexpected field, with name " << minuend.fields[i].name << ". This field will be stripped from the output.");
        }
        ROS_DEBUG_STREAM("[PointcloudSubtraction] Minuend fields - RGB: " << minuend_rgb+0 << " Intensity:" << minuend_intensity+0 << " Normals: " << minuend_normals);

        // ------------ SUBTRAHEND ------------
        bool subtrahend_rgb = false;           // Does the subtrahend cloud contain RGB data??
        bool subtrahend_intensity = false;     // Does the subtrahend cloud contain INTENSITY data? 
        bool subtrahend_normals = false;       // Does the subtrahend cloud contain NORMALS data? 
        for(int i=0; i<subtrahend.fields.size(); i++)
        {
        	std::string name = subtrahend.fields[i].name;
            if(name.compare("rgb") == 0)
                subtrahend_rgb = true;
            else if(name.compare("intensity") == 0)
                subtrahend_intensity = true;
            else if(name.compare("normal_x") == 0)        // although clouds with normal data actually devote three fields to it (normal_x, normal_y, normal_z, and curvature)...
                subtrahend_normals = true;
            else if(name.compare("normal_y") != 0 && name.compare("normal_z") != 0 && name.compare("curvature") != 0) 	// Field is not a Normal field
        		if(name.compare("x") != 0 && name.compare("y") != 0 && name.compare("z") != 0)							// Field is not an XYZ field
                    ROS_WARN_STREAM("[PointcloudSubtraction] During cloud subtraction, subtrahend cloud contains an unexpected field, with name " << subtrahend.fields[i].name << ". This field will be stripped from the output.");
        }
        ROS_DEBUG_STREAM("[PointcloudSubtraction] Subtrahend fields - RGB: " << subtrahend_rgb+0 << " Intensity: " << subtrahend_intensity+0 << " Normals: " << subtrahend_normals);

        // ------------------------- ARE FIELDS CONSISTENT WITH CHECKS REQUESTED? -----------------------------
        // This will ONLY 'fail' if a field check is requested, and that field exists in one cloud but NOT the other
        // Not sure if I will keep it this way in the long term... might make more sense to be either more or less strict?

        // ------------ RGB CHECK ------------
        if(check_rgb)
        {
            if( minuend_rgb+subtrahend_rgb )                // At least one cloud contains rgb information
            {
                if( !(minuend_rgb*subtrahend_rgb) )             // BUT * NOT BOTH *! That's a problem
                {
                    ROS_WARN_STREAM("[PointcloudSubtraction] Cloud subtraction requested with an rgb check, but only one cloud contains rgb information. Minuend rgb: " << minuend_rgb+0 << " Subtrahend rgb: " << subtrahend_rgb+0 << ". Returning minuend...");
                    return minuend;
                }
            }
            else                                            // Neither cloud has rgb information
                check_rgb = false;
        }
        // ------------ INTENSITY CHECK ------------
        if(check_intensity)
        {
            if( minuend_intensity+subtrahend_intensity )    // At least one cloud contains intensity information
            {
                if( !(minuend_intensity*subtrahend_intensity) ) // BUT * NOT BOTH *! That's a problem
                {
                    ROS_WARN_STREAM("[PointcloudSubtraction] Cloud subtraction requested with an intensity check, but only one cloud contains intensity information. Minuend intensity: " << minuend_intensity+0 << " Subtrahend intensity: " << subtrahend_intensity+0 << ". Returning minuend...");
                    return minuend;
                }
            }
            else                                            // Neither cloud has intensity information
                check_intensity = false;
        }
        // ------------ NORMALS CHECK ------------
        if(check_normals)
        {
            if( (minuend_normals+subtrahend_normals) )      // At least one cloud contains normals information
            {
                if( !(minuend_normals*subtrahend_normals))      // BUT * NOT BOTH *! That's a problem
                {
                    ROS_WARN_STREAM("[PointcloudSubtraction] Cloud subtraction requested with an normals check, but only one cloud contains intensity information. Minuend normals: " << minuend_normals+0 << " Subtrahend normals: " << subtrahend_normals+0 << ". Returning minuend...");
                    return minuend;                    
                }
            }
            else 
                check_normals = false;                      // Neither cloud has normals information
        }
        ROS_DEBUG_STREAM("[PointcloudSubtraction] After looking at the data the clouds actually contain, check options are updated to - CHECK_RGB: " << check_rgb+0 << " CHECK_INTENSITY: " << check_intensity+0 << " CHECK_NORMALS: " << check_normals+0);

        // ------------------------- CALL LOWER-LEVEL TEMPLATED FUNCTION -----------------------------
        // Again, lower-leve functions are templated because PCL clouds are templated on point types
        // This structure isolates the ROS user from PCL templates
        
        if(minuend_rgb || subtrahend_rgb)
        {
            if(minuend_normals || subtrahend_normals)
                return templatedSubtraction<pcl::PointXYZRGBNormal>(minuend, subtrahend, check_rgb, check_intensity, check_normals, min_dist, num_neighbors_compared);      // XYZ, RGB, Normals
            return templatedSubtraction<pcl::PointXYZRGB>(minuend, subtrahend, check_rgb, check_intensity, check_normals, min_dist, num_neighbors_compared);                 // XYZ, RGB
        }
        if(minuend_intensity || subtrahend_intensity)                                           
        {
            if(minuend_normals || subtrahend_normals)
                return templatedSubtraction<pcl::PointXYZINormal>(minuend, subtrahend, check_rgb, check_intensity, check_normals, min_dist, num_neighbors_compared);        // XYZ, I, Normals
            return templatedSubtraction<pcl::PointXYZI>(minuend, subtrahend, check_rgb, check_intensity, check_normals, min_dist, num_neighbors_compared);                   // XYZ, I
        }
        if(minuend_normals || subtrahend_normals)
            return templatedSubtraction<pcl::PointNormal>(minuend, subtrahend, check_rgb, check_intensity, check_normals, min_dist, num_neighbors_compared);                // XYZ, Normals
        return templatedSubtraction<pcl::PointXYZ>(minuend, subtrahend, check_rgb, check_intensity, check_normals, min_dist, num_neighbors_compared);                         // XYZ            
    
    } 	// subtractClouds()
    // ----------------------------------------------------------------------------------------------------------------------

}