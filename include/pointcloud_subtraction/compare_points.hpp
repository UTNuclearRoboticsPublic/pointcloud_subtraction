namespace PointcloudSubtraction
{
    // All the below functions are templated on different PCL PointTypes
    // Having a field (like RGB, Intensity, or Normals) does not mean that field will be actually used in the point comparison
    // That choice is set by the check_rgb, check_intensity, and check_normals parameters
    // When the top-level function used is the non-templated ROS Msg version, these booleans can be assigned by default values
    //  which are ALL TRUE - so by default the system tries to check all the fields it can 

    template<>
    bool comparePoints<pcl::PointXYZ>(pcl::PointXYZ minuend_point, pcl::PointXYZ subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	return true;
    }

    template<>
    bool comparePoints<pcl::PointXYZI>(pcl::PointXYZI minuend_point, pcl::PointXYZI subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	if(check_intensity)
    		if(minuend_point.intensity == subtrahend_point.intensity)
    			return true;
    		else
    			return false;
		return true;
    }

    template<>
    bool comparePoints<pcl::PointNormal>(pcl::PointNormal minuend_point, pcl::PointNormal subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	if(check_normals)
    	{
    		if(minuend_point.normal_x == subtrahend_point.normal_x)
    			if(minuend_point.normal_y == subtrahend_point.normal_y)
    				if(minuend_point.normal_z == subtrahend_point.normal_z)
    					if(minuend_point.curvature == subtrahend_point.curvature)
    						return true;
			return false;
    	}
		return true;
    }

    template<>
    bool comparePoints<pcl::PointXYZINormal>(pcl::PointXYZINormal minuend_point, pcl::PointXYZINormal subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	if(check_intensity)
    		if(minuend_point.intensity != subtrahend_point.intensity)
    			return false;
    	if(check_normals)
    	{
    		if(minuend_point.normal_x == subtrahend_point.normal_x)
    			if(minuend_point.normal_y == subtrahend_point.normal_y)
    				if(minuend_point.normal_z == subtrahend_point.normal_z)
    					if(minuend_point.curvature == subtrahend_point.curvature)
    						return true;
			return false;
    	}
		return true;	
    }

    template<>
    bool comparePoints<pcl::PointXYZRGB>(pcl::PointXYZRGB minuend_point, pcl::PointXYZRGB subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	if(check_rgb)
    	{
    		if(minuend_point.r == subtrahend_point.r)
    			if(minuend_point.g == subtrahend_point.g)
    				if(minuend_point.b == subtrahend_point.b)
    					return true;
			return false;
    	}
		return true;
    }

    template<>
    bool comparePoints<pcl::PointXYZRGBNormal>(pcl::PointXYZRGBNormal minuend_point, pcl::PointXYZRGBNormal subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	if(check_rgb)
    	{
    		if(minuend_point.r != subtrahend_point.r)
    			return false;
    		if(minuend_point.g != subtrahend_point.g)
    			return false;
			if(minuend_point.b != subtrahend_point.b)
				return false;
    	}
    	if(check_normals)
    	{
    		if(minuend_point.normal_x == subtrahend_point.normal_x)
    			if(minuend_point.normal_y == subtrahend_point.normal_y)
    				if(minuend_point.normal_z == subtrahend_point.normal_z)
    					if(minuend_point.curvature == subtrahend_point.curvature)
    						return true;
			return false;
    	}
		return true;
    }
}