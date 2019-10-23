/* \author Aaron Brown */
// Create simple 3d highway environment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "PCLprocessPointClouds.h"

// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "PCLprocessPointClouds.cpp"

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

    auto lidar = std::make_shared<Lidar>(cars, 0);
    auto inputCloud = lidar->scan();
    //renderPointCloud(viewer, inputCloud, "initial_points");
    
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    auto segmentCloud = pointProcessor.SegmentPlane(inputCloud, 150, 0.25);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud", Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(1,1,1));

    auto cloudClusters = pointProcessor.Clustering(segmentCloud.first, 2.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0)
                                 , Color(0,0,1), Color(1,1,0)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId, colors[clusterId % colors.size()],0.5);
        ++clusterId;
    }
}


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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(
            inputCloud, 0.25 , Eigen::Vector4f (-10, -5, -3, 1),
            Eigen::Vector4f ( 20, 7, 0.5, 1));
    renderPointCloud(viewer,filterCloud,"filterCloud");
    std::vector<Color> colors = {Color(1,0,0),
                                 Color(1,1,0),
                                 Color(0,1,0),
                                 Color(0,1,1),
                                 Color(0,0,1)};

    auto segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.3);

    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",colors[0]);
    renderPointCloud(viewer,segmentCloud.second,"planeCloud", Color(1,1,1));

    auto cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 20, 1000);

    int clusterId = 0;
    for(auto cluster : cloudClusters)
    {
        std::cout << clusterId << " cluster size ";
        pointProcessorI->numPoints(cluster);
        //renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[4]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId, colors[0],0.5);
        ++clusterId;
    }

}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
//    /*
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");

    auto streamIterator = stream.begin();
//    */
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    /*
    inputCloudI = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    cityBlock(viewer, pointProcessorI, inputCloudI);
    //*/
    while (!viewer->wasStopped ())
    {
//    /*
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();


        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
//        */
        viewer->spinOnce ();
    }
}