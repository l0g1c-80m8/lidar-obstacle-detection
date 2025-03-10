/* \author Rutvik patel */
#pragma once

#include "render/render.h"
#include "render/box.h"
#include <chrono>
#include <string>
#include "kdtree_3d.h"

void clusterHelper(int idx, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree3D* tree, float distanceTol) {
	if (processed[idx]) return;

  	processed[idx] = true;
    cluster.push_back(idx);

    std::vector<int> nearest = tree->search(points[idx], distanceTol);

    for (auto id : nearest) {
    	if (!processed[id])
          clusterHelper(id, points, cluster, processed, tree, distanceTol);
    }
}

std::vector<std::vector<int>> euclideanCluster3D(const std::vector<std::vector<float>>& points, KdTree3D* tree, float distanceTol)
{

	// Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    int idx = 0;

    while (idx < points.size()) {
    	if (!processed[idx]) {
    		std::vector<int> cluster;
    		clusterHelper(idx, points, cluster, processed, tree, distanceTol);
    		clusters.push_back(cluster);
    	}

        ++idx;
    }
 
	return clusters;

}
