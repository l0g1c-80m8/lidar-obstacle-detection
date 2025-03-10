/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

//std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
//{
//	std::unordered_set<int> inliersResult;
//	srand(time(NULL));
//
//	// Fill in this function
//
//	// For max iterations
//    while (maxIterations--) {
//		// Randomly sample subset and fit line
//        std::unordered_set<int> inliers;
//        while (inliers.size() < 2) { inliers.insert(rand() % cloud->points.size()); }
//
//    	// Measure distance between every point and fitted line
//        auto itr = inliers.begin();
//        float x1 = cloud->points[*itr].x;
//        float y1 = cloud->points[*itr].y;
//        itr++;
//    	float x2 = cloud->points[*itr].x;
//    	float y2 = cloud->points[*itr].y;
//
//        float a = y1 - y2;
//        float b = x2 - x1;
//        float c = x1 * y2 - x2 * y1;
//
//        for (auto idx = 0; idx < cloud->points.size(); ++idx) {
//        	if (inliers.count(idx) > 0) continue;
//            pcl::PointXYZ point = cloud->points[idx];
//            float x3 = point.x;
//         	float y3 = point.y;
//            float d = fabs(a * x3 + b * y3 + c) / sqrt(a * a + b * b);
//
//        	// If distance is smaller than threshold count it as inlier
//            if (d <= distanceTol) inliers.insert(idx);
//        }
//
//        if (inliers.size() > inliersResult.size()) inliersResult = inliers;
//    }
//
//	// Return indicies of inliers from fitted line with most inliers
//	return inliersResult;
//
//}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // Iterate for the maximum number of iterations
    while (maxIterations--) {
        // Randomly sample three distinct points
        std::unordered_set<int> inliers;
        while (inliers.size() < 3) {
            inliers.insert(rand() % cloud->points.size());
        }

        auto itr = inliers.begin();
        pcl::PointXYZ p1 = cloud->points[*itr];
        itr++;
        pcl::PointXYZ p2 = cloud->points[*itr];
        itr++;
        pcl::PointXYZ p3 = cloud->points[*itr];

        // Compute vectors v1 and v2
        float v1x = p2.x - p1.x;
        float v1y = p2.y - p1.y;
        float v1z = p2.z - p1.z;

        float v2x = p3.x - p1.x;
        float v2y = p3.y - p1.y;
        float v2z = p3.z - p1.z;

        // Compute cross product to get plane normal (A, B, C)
        float A = v1y * v2z - v1z * v2y;
        float B = v1z * v2x - v1x * v2z;
        float C = v1x * v2y - v1y * v2x;
        float D = -(A * p1.x + B * p1.y + C * p1.z);

        // Ensure the plane is valid (avoid division by zero)
        float norm = sqrt(A * A + B * B + C * C);
        if (norm == 0) continue;

        // Iterate through all points to find inliers
        for (int idx = 0; idx < cloud->points.size(); idx++) {
            if (inliers.count(idx) > 0) continue;  // Skip already selected points

            pcl::PointXYZ point = cloud->points[idx];
            float d = fabs(A * point.x + B * point.y + C * point.z + D) / norm;

            // If the distance is within tolerance, count it as an inlier
            if (d <= distanceTol) {
                inliers.insert(idx);
            }
        }

        // Update the best inlier set if more inliers are found
        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

    return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
