/* \author Rutvik Patel */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
// #include "processPointClouds.cpp"

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
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor
    auto* lidar = new Lidar(cars, 0.0);
    auto scan = lidar->scan();
    // renderRays(viewer, lidar->position, scan);
    // renderPointCloud(viewer, scan, "lidar scan");

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    auto segmentCloud = pointProcessor.SegmentPlane(scan, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstacle cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "plane cloud", Color(0, 1, 0));

    auto cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (auto clusterCloud : cloudClusters) {
        std::cout << "cluster size";
        pointProcessor.numPoints(clusterCloud);
        renderPointCloud(viewer, clusterCloud, "obstacle cloud " + std::to_string(clusterId), colors[clusterId % colors.size()]);

        auto box = pointProcessor.BoundingBox(clusterCloud);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}

// use custom segment plane and clustering implementations for cityBlock
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    inputCloud = pointProcessor->FilterCloud(inputCloud, 0.4, Eigen::Vector4f(-10, -6, -2, 1), Eigen::Vector4f(30, 6, 1, 1));
    auto segmentCloud = pointProcessor->SegmentPlaneCustom(inputCloud, 20, 0.3);
    renderPointCloud(viewer, segmentCloud.first, "obstacle cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "plane cloud", Color(0, 1, 0));
    auto cloudClusters = pointProcessor->ClusteringCustom(segmentCloud.first, 0.53, 10, 300);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    for (auto clusterCloud : cloudClusters) {
        std::cout << "cluster size";
        pointProcessor->numPoints(clusterCloud);
        renderPointCloud(viewer, clusterCloud, "obstacle cloud " + std::to_string(clusterId), colors[clusterId % colors.size()]);

        auto box = pointProcessor->BoundingBox(clusterCloud);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}


// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
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


int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // simpleHighway(viewer);
    auto* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloud = pointProcessor->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessor, inputCloud);

        // Updates streamIterator. Loops from beginning if reach end.
        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}