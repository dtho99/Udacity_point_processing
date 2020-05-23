/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0);

    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr = lidar->scan();
    //renderRays(viewer, 	Vect3(0, 0, 0), pointCloudPtr);
    //renderRays(viewer, 	lidar->position, pointCloudPtr);    

    renderPointCloud(viewer, pointCloudPtr, "Cloud");

    // create a point processor
    ProcessPointClouds<pcl::PointXYZ> processPointClouds;   

    // processPointClouds ptr can also be created as follows
    //    ProcessPointClouds<pcl::PointXYZ> *processPointClouds = new ProcessPointClouds<pcl::PointXYZ>();  

    // segmenting extracts outliers (objects) from inliers (road) using RANSAC to identify inliers
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processPointClouds.SegmentPlane(pointCloudPtr, 500, 0.5);

    renderPointCloud(viewer, segmentCloud.first, "obstacle", Color(1,0,0));
    std::cout << "obstacle cloud size " << segmentCloud.first->size() << "inlier size" << segmentCloud.second->size() << std::endl;

    renderPointCloud(viewer, segmentCloud.second, "plane", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processPointClouds.Clustering(segmentCloud.first, 1, 3, 30);

    int clusterId = 0;
    std::vector<Color> color = {Color(0,1,1), Color(1,1,0), Color(0,0,1), Color(1,0,1)};
  
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        processPointClouds.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacle " + std::to_string(clusterId), color[clusterId % sizeof(color)]);

        Box box = processPointClouds.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        clusterId++;
    }

}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, 
               ProcessPointClouds<pcl::PointXYZI> *processPointClouds, 
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloudPtr)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;


    // processPointClouds ptr can also be created as follows from the heap
    //ProcessPointClouds<pcl::PointXYZI> *processPointClouds = new ProcessPointClouds<pcl::PointXYZI>();  
    //pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr = 
    //            processPointClouds->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    //Eigen::Vector4f minCrop(100,10,2, 1);
    //Eigen::Vector4f maxCrop = new Eigen::Vector(-20,-10,-10, 1);
    // roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));

    float filterRes = 0.1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloudPtr = 
        processPointClouds->FilterCloud(pointCloudPtr, filterRes, 
            Eigen::Vector4f(-10,-5,-10,1),
            Eigen::Vector4f(50,8,1,1)); 


 
    //renderPointCloud(viewer, filterCloudPtr, "plane");

    printf("Filter Num points %ld, Non Filter %ld, \n", filterCloudPtr->points.size(), pointCloudPtr->points.size());


    // segmenting extracts outliers (objects) from inliers (road) using RANSAC to identify inliers
    int maxIterations = 100;
    float distanceThreshold = 0.4;  // deviation from the plane to be considered an outlier
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = 
                            processPointClouds->SegmentPlane(filterCloudPtr, maxIterations, distanceThreshold);

    renderPointCloud(viewer, segmentCloud.first, "obstacle", Color(1,0,0));
    std::cout << "obstacle cloud size " << segmentCloud.first->size() << "inlier size" << segmentCloud.second->size() << std::endl;

    //renderPointCloud(viewer, segmentCloud.second, "plane", Color(0,1,0));

    // cluster (kdTree)
    float clusterTolerance = 0.8;
    int minSize = 15; 
    int maxSize = 1800;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = 
                        processPointClouds->Clustering(segmentCloud.first, clusterTolerance, minSize, maxSize);

    int clusterId = 0;
    std::vector<Color> color = {Color(0,1,1), Color(1,1,0), Color(0,0,1), Color(1,0,1)};
  
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
      
        processPointClouds->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstatcle " + std::to_string(clusterId), color[clusterId % 4]);

        Box box = processPointClouds->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, color[clusterId % 4]);

        ++clusterId;
    }
   

}



/*
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;


    // processPointClouds ptr can also be created as follows from the heap
    ProcessPointClouds<pcl::PointXYZI> *processPointClouds = new ProcessPointClouds<pcl::PointXYZI>();  
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr = 
                processPointClouds->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    //Eigen::Vector4f minCrop(100,10,2, 1);
    //Eigen::Vector4f maxCrop = new Eigen::Vector(-20,-10,-10, 1);
    // roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));

    float filterRes = 0.1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloudPtr = 
        processPointClouds->FilterCloud(pointCloudPtr, filterRes, 
            Eigen::Vector4f(-10,-5,-10,1),
            Eigen::Vector4f(50,8,1,1)); 


 
    //renderPointCloud(viewer, filterCloudPtr, "plane");

    printf("Filter Num points %ld, Non Filter %ld, \n", filterCloudPtr->points.size(), pointCloudPtr->points.size());


    // segmenting extracts outliers (objects) from inliers (road) using RANSAC to identify inliers
    int maxIterations = 100;
    float distanceThreshold = 0.4;  // deviation from the plane to be considered an outlier
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = 
                            processPointClouds->SegmentPlane(filterCloudPtr, maxIterations, distanceThreshold);

    renderPointCloud(viewer, segmentCloud.first, "obstacle", Color(1,0,0));
    std::cout << "obstacle cloud size " << segmentCloud.first->size() << "inlier size" << segmentCloud.second->size() << std::endl;

    //renderPointCloud(viewer, segmentCloud.second, "plane", Color(0,1,0));

    // cluster (kdTree)
    float clusterTolerance = 0.8;
    int minSize = 15; 
    int maxSize = 1500;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = 
                        processPointClouds->Clustering(segmentCloud.first, clusterTolerance, minSize, maxSize);

    int clusterId = 0;
    std::vector<Color> color = {Color(0,1,1), Color(1,1,0), Color(0,0,1), Color(1,0,1)};
  
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
      
        processPointClouds->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstatcle " + std::to_string(clusterId), color[clusterId % 4]);

        Box box = processPointClouds->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, color[clusterId % 4]);

        ++clusterId;
    }
}
*/



//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
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

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle =  XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI> *processPointClouds = new ProcessPointClouds<pcl::PointXYZI>();  
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

//    std::vector<boost::filesystem::path> stream = processPointClouds->streamPcd("../src/sensors/data/pcd/data_1");
    std::vector<boost::filesystem::path> stream = processPointClouds->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
    
 
     
    //cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        // clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        // load pcd 
        inputCloud = processPointClouds->loadPcd((*streamIterator).string());
        cityBlock(viewer, processPointClouds, inputCloud);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}