/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "parameters.h"


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


// void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
// {
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display simple highway -----
//     // ----------------------------------------------------
//
//     // RENDER OPTIONS
//     bool renderScene = false;
//     std::vector<Car> cars = initHighway(renderScene, viewer);
//
//     // Create lidar sensor
//     Lidar* lidar = new Lidar(cars, 0);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
//     ProcessPointClouds<pcl::PointXYZ>* pointCloudProcessor = new ProcessPointClouds<pcl::PointXYZ>();
//     std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointCloudProcessor->SegmentPlane(cloud, 100, 0.2);
//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointCloudProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
//     int clusterId = 0;
//     std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
//
//     for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
//     {
//           std::cout << "cluster size ";
//           pointCloudProcessor->numPoints(cluster);
//           renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
//           Box boundingBox = pointCloudProcessor->BoundingBox(cluster);
//           renderBox(viewer, boundingBox, clusterId, Color(1,0,1), 0.5);
//           ++clusterId;
//     }
//
//
//
// }

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  // Read from `params` file for tuning filter values at runtime
  auto minPointVec = Parameters::getValues<float>(MIN_POINT);
  auto maxPointVec = Parameters::getValues<float>(MAX_POINT);
  float minPointarr[4];
  std::copy(minPointVec.begin(), minPointVec.end(), minPointarr);
  float maxPointarr[4];
  std::copy(maxPointVec.begin(), maxPointVec.end(), maxPointarr);

  Eigen::Vector4f minPoint(minPointarr);
  Eigen::Vector4f maxPoint(maxPointarr);


  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, Parameters::getValues<float>(FILTER_RES)[0] , minPoint, maxPoint);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, Parameters::getValues<int>(MAX_ITER)[0], Parameters::getValues<float>(DISTANCE_THRESHOLD)[0], Parameters::getValues<bool>(USE_PCL_RANSAC)[0]);
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, Parameters::getValues<float>(CLUSTER_TOL)[0], Parameters::getValues<int>(CLUSTER_MIN)[0], Parameters::getValues<int>(CLUSTER_MAX)[0], Parameters::getValues<bool>(USE_PCL_EC)[0]);
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,0,1), Color(1,1,0)};

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        Box boundingBox = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, boundingBox, clusterId, Color(1,0,1), 0.5);
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




int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(Parameters::getValues<string>(PCD_FOLDER)[0]);
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    while (!viewer->wasStopped ())
    {

      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);

      streamIterator++;
      if(streamIterator == stream.end())
        streamIterator = stream.begin();

      viewer->spinOnce ();
    }
}
