// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "quiz/cluster/kdtree.h"
#include <unordered_set>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
        typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    typename pcl::PointCloud<PointT>::Ptr sortedCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr croppedCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr withoutRoof(new pcl::PointCloud<PointT>());

    sor.filter (*sortedCloud);

    pcl::CropBox<PointT> cropBox;
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);

    cropBox.setInputCloud(sortedCloud);
    cropBox.filter(*croppedCloud);

    pcl::CropBox<PointT> cropRoof;
    cropRoof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    cropRoof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    cropRoof.setInputCloud(croppedCloud);
    cropRoof.setNegative(true);
    cropRoof.filter(*croppedCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return croppedCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(new pcl::PointCloud<PointT>, new pcl::PointCloud<PointT>);
    std::cerr << "PointCloud representing the all input: " << cloud->points.size() << " data points."  << std::endl;

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*(segResult.first));
    std::cerr << "PointCloud representing the obstacles: " << segResult.first->points.size() << " data points."  << std::endl;

    // Create the filtering object
    extract.setNegative (false);
    extract.filter (*(segResult.second));
    std::cerr << "PointCloud representing the planar component: "<< segResult.second->points.size() << " data points." << std::endl;

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers( new pcl::PointIndices());

    std::unordered_set<int> bestResult;
    srand(time(NULL));
    // For max iterations
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        std::unordered_set<int> result;

        while (result.size() < 3) {
            result.insert(rand() % cloud->points.size());
        }
        auto it = result.begin();
        const PointT & p1 = cloud->points[*it++];
        const PointT & p2 = cloud->points[*it++];
        const PointT & p3 = cloud->points[*it++];

        float A = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
        float B = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
        float C = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
        float D = -(A * p1.x + B * p1.y + C * p1.z);
        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        for (int i = 0; i < cloud->size(); ++i) {
            if (result.count(i)) continue;

            const PointT & p = cloud->at(i);
            float d = fabs(A * p.x + B * p.y + C * p.z + D) / sqrt(A * A + B * B + C * C);

            if (d < distanceThreshold) result.insert(i);
        }
        if (result.size() > bestResult.size())
            bestResult = result;
    }

    for (int p: bestResult)
        inliers->indices.push_back(p);


    if (inliers->indices.empty())
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void fillTree( KdTree<PointT> & tree, typename pcl::PointCloud<PointT>::Ptr cloud,
        std::vector<int>::iterator indices_begin, std::vector<int>::iterator indices_end, int depth )
{
    if (indices_end - indices_begin < 1) return;

    std::sort(indices_begin, indices_end, [cloud, depth](const int & pointIndex1, const int & pointIndex2)
    {
        return cloud->points[pointIndex1].data[depth % N_DIMENSIONS] < cloud->points[pointIndex2].data[depth % N_DIMENSIONS];
    });

    auto indices_median = indices_begin + (indices_end - indices_begin) / 2;
    tree.insert(cloud->points[*indices_median], *indices_median);

    fillTree(tree, cloud, indices_begin, indices_median, depth + 1);
    fillTree(tree, cloud, indices_median + 1, indices_end, depth + 1);

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree<PointT> tree;

    std::vector <int> indices(cloud->points.size());
    std::iota(indices.begin(), indices.end(), 0);
    fillTree(tree, cloud, indices.begin(), indices.end(), 0);

    auto cluster_indices = euclideanCluster(cloud,tree,clusterTolerance);

    for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        if (it->size() < minSize || it->size() > maxSize)
            continue;

        for (auto pit : *it)
            cloud_cluster->points.push_back (cloud->points[pit]);

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


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


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


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


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}