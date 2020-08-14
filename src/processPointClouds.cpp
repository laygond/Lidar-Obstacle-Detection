// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


/**
* Displays in console the number of points (size) of input cloud 
*/
template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


/**
 * Apply Voxel Grid Filtering and extract Region of Interest
 * Returns downsampled cloud with only points inside the RoI
 *  
 *     @param cloud     : input cloud
 *     @param filterRes : voxel grid size,
 *     @param minPoint  : min points of region of interest.
 *     @param maxPoint  : max points of region of interest.
 *     
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:
    // Create voxel grid point reduction object: downsample the dataset using a leaf size of filterRes
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>() );
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes); //voxel's dimension in [m]
    vg.filter (*cloud_filtered);

    // Region based filtering
    pcl::CropBox<PointT> region_box(false); // true means extract indeces from points outside box
    region_box.setMin(minPoint);
    region_box.setMax(maxPoint);
    region_box.setInputCloud(cloud_filtered);
    region_box.filter(*cloud_filtered);

    // Remove Points from Roof
    std::vector<int> indeces; 
    pcl::CropBox<PointT> roof_box(true); 
    roof_box.setMin(Eigen::Vector4f (-1.5 ,-1.7 ,-1  ,1));
    roof_box.setMax(Eigen::Vector4f (2.6  , 1.7 ,-.4 ,1));
    roof_box.setInputCloud(cloud_filtered);
    roof_box.filter(indeces);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int idx: indeces) 
       inliers->indices.push_back(idx);
    
    pcl::ExtractIndices<PointT> extract;      // Create the filtering object
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (true);   //extract points different than inliers (if false then it extracts inliers)
    extract.filter (*cloud_filtered);

    // Calculate Time of Filtering
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;
}


/**
* Separates an input cloud in two groups given the indeces of first group 
*/
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with segmented plane and other with obstacles 
    typename pcl::PointCloud<PointT>::Ptr cloud1 (new pcl::PointCloud<PointT>() );
    typename pcl::PointCloud<PointT>::Ptr cloud2 (new pcl::PointCloud<PointT>() );
    
    //Get first cloud group from given indeces
    for (const auto& idx: inliers->indices)
       cloud1->points.push_back(cloud->points[idx]);

    // Extract from input cloud second group using pcl extract
    pcl::ExtractIndices<PointT> extract;      // Create the filtering object
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);   // points different than inliers (if false then it extracts first group)
    extract.filter (*cloud2);

    // Create pair and return
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud1, cloud2);
    return segResult;
}


/**
* Apply RANSAC: the segmentation algorithm fits a plane to the points and uses the distance tolerance
* to decide which points belong to that plane. A larger tolerance includes more points in the plane.
* This is an iterative process, more iterations have a chance to return better results but take longer.
*
*/
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO: 
    // Create the segmentation object, coefficients, and inliers
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    
    //Set Segmentation Parameters
    seg.setOptimizeCoefficients (true); // Optional
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    //Segment largest planar component from input cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cout<<"Could not estimate a planar model for the given dataset."<<std::endl;
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    // Calculate segmentation time
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


/**
* Euclidean Clustering: associate groups of points by how close together they are.
*
* @param distance tolerance.: Any points within that distance will be grouped together.
* @param min: the idea is: if a cluster is really small, it’s probably just noise.
* @param max: number of points allows us to better break up very large clusters.
*/
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // TODO
    // Create vector to store euclidean clustered groups
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    //Create a Cluster extraction object and vector of cluster indeces 
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); //[m]
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    //Assign clustered indices to a point cloud object and append to 'clusters' vector 
    for (pcl::PointIndices it: clusterIndices) //[it]eration
    {
        typename pcl::PointCloud<PointT>::Ptr aCloudCluster (new pcl::PointCloud<PointT>);
        for (int index : it.indices)
            aCloudCluster->push_back (cloud->points[index]);
        aCloudCluster->width = aCloudCluster->size ();
        aCloudCluster->height = 1;
        aCloudCluster->is_dense = true;

        clusters.push_back(aCloudCluster);
    }

    // Calculate clustering time
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

/**
* Set Box Coodinates of a point cloud so that it can be rendered
*/
template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

/**
* Save Point Cloud to File
*/
template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

/**
* Load Single Point Cloud Data File
*/
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

/**
* Stream Chronologically Point Cloud Data from directory path
*/
template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in ascending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}