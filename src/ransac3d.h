/* \author Rutvik Patel */
#pragma once

#include <unordered_set>
#include <pcl/common/common.h>

template <typename PointT> 
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
        PointT p1 = cloud->points[*itr];
        itr++;
        PointT p2 = cloud->points[*itr];
        itr++;
        PointT p3 = cloud->points[*itr];

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
        if (norm < 1e-6) continue;

        // Iterate through all points to find inliers
        for (int idx = 0; idx < cloud->points.size(); idx++) {
            if (inliers.count(idx) > 0) continue;  // Skip already selected points

            PointT point = cloud->points[idx];
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
