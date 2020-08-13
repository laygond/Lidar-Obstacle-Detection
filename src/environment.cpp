/* \author Aaron Brown */
// Modified by Bryan Laygond
// Simple 3d enviroment using PCL for exploring self-driving car sensors
//      simpleHighway: uses simulated Point Cloud Data
//      cityBlock    : uses real Point Cloud Data

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


// ----------------------------------------------------
// -------------- Simple Highway ----------------------
// ----------------------------------------------------

/**
 * Creates a highway and cars to later render it in a viewer
 */
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

/**
 * Scan, Segment, Cluster, and Display in Viewer (XYZ Lidar PCD)
 */
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{  
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:
    // Create simulated lidar sensor and scan environment
    Lidar* lidar = new Lidar(cars,0);       //must notify surrounding objects, location of objects in environment 
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan(); // since it knows locations of objects it knows how to scan
    //renderRays(viewer, lidar->position, inputCloud); // display rays from the lidar as the origin
    //renderPointCloud(viewer, inputCloud,"inputCloud",Color(1,0,1));

    // Create point processor
    //  stack version
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    //  heap version
    //ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    // Segment Obstacles and Plane
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    //renderPointCloud(viewer,segmentCloud.first,"planeCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"obstCloud",Color(0,1,0));

    //Cluster points in obstacle cloud to assign boxes
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.second, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        //Box box = pointProcessor.BoundingBox(cluster);
        //renderBox(viewer,box,clusterId);        
        ++clusterId;
    }
}


// ----------------------------------------------------
// ------------------ City Block  ---------------------
// ----------------------------------------------------

/**
 * Load, Filter, Segment, Cluster, Display in Viewer (XYZI Lidar PCD)
 */
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    //Create point processor
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    // Read/Load Point Cloud Data
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer,inputCloud,"inputCloud");   // if the color is not specified in the renderPointCloud function argument, it will default to using the intensity color coding.

    // Filter
    float voxelCubeSize = .2; //[m]
    Eigen::Vector4f cropMinPoint(-15.0, -5  , -4  , 1.0);
    Eigen::Vector4f cropMaxPoint( 30  ,  8  ,  4  , 1.0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, voxelCubeSize, cropMinPoint, cropMaxPoint);
    //renderPointCloud(viewer,filterCloud,"filterCloud");

    // Segment Obstacles and Plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    //renderPointCloud(viewer,segmentCloud.first,"planeCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"obstCloud",Color(0,1,0));

    //Cluster points in obstacle cloud to assign boxes
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.second, 2.0, 3, 600);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1), Color(1,1,1), Color(1,1,0), Color(0,.5,0.8), Color(0.5,0,1), Color(0.5,0.5,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        //Box box = pointProcessor.BoundingBox(cluster);
        //renderBox(viewer,box,clusterId);        
        ++clusterId;
    }
}


// ----------------------------------------------------
// -------------------  General  ----------------------
// ----------------------------------------------------

/**
 * Switch Camera Angle in Viewer
 * setAngle: {XY, TopDown, Side, FPS}
 */
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    // Create Viewer and set Angle
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // Render in Viewer
    //simpleHighway(viewer); // Uses simulated Point Cloud Data
    cityBlock(viewer);       // Uses real Point Cloud Data

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}