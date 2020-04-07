// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::VoxelGrid<PointT> voxel_filter;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(filterRes, filterRes, filterRes);
    voxel_filter.filter(*cloud_filtered);

    pcl::CropBox<PointT> box_filter;
    box_filter.setMin(minPoint);
    box_filter.setMax(maxPoint);
    box_filter.setInputCloud(cloud_filtered);
    box_filter.filter(*cloud_filtered);

    pcl::CropBox<PointT> body_filter;
    body_filter.setMin(Eigen::Vector4f(-2.0, -2.0, -4.0, 1));
    body_filter.setMax(Eigen::Vector4f(3.0, 2.0, 2.0, 1));
    body_filter.setNegative(true);
    body_filter.setInputCloud(cloud_filtered);
    body_filter.filter(*cloud_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr ground_plane(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*ground_plane);

    extract.setNegative(true);
    extract.filter(*obstacles);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, ground_plane);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    int i = 0, nr_points = (int)cloud->points.size();
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(const typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    int numPoints = cloud->points.size();
    std::srand(std::time(nullptr));
    // For max iterations
    for (int i = 0; i < maxIterations; i++)
    {
        // Randomly sample subset and fit line
        int index_1 = (rand() % numPoints) + 1;
        int index_2 = (rand() % numPoints) + 1;
        int index_3 = (rand() % numPoints) + 1;
        while (index_1 == index_2 || index_2 == index_3 || index_3 == index_1)
        {
            index_1 = (rand() % numPoints) + 1;
            index_2 = (rand() % numPoints) + 1; // lazy way to make sure we get 3 different pts
        }
        auto pt1 = cloud->points[index_1];
        auto pt2 = cloud->points[index_2];
        auto pt3 = cloud->points[index_3];

        Eigen::Vector3d vector1(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z);
        Eigen::Vector3d vector2(pt3.x - pt1.x, pt3.y - pt1.y, pt3.z - pt1.z);

        Eigen::Vector3d normal_vector;
        normal_vector = vector1.cross(vector2);

        float A = normal_vector[0];
        float B = normal_vector[1];
        float C = normal_vector[2];
        float D = -(A * pt1.x + B * pt1.y + C * pt1.z);

        float A_squared = std::pow(A, 2);
        float B_squared = std::pow(B, 2);
        float C_squared = std::pow(C, 2);
        float denominator = std::sqrt(A_squared + B_squared + C_squared);

        // Measure distance between every point and fitted line
        std::unordered_set<int> inliersCandidate;
        for (int j = 0; j < numPoints; j++)
        {
            auto candidatePt = cloud->points[j];
            float distance = std::abs(A * candidatePt.x + B * candidatePt.y + C * candidatePt.z + D) / denominator;
            if (distance <= distanceTol)
            {
                // If distance is smaller than threshold count it as inlier
                inliersCandidate.insert(j);
            }
        }
        if (inliersCandidate.size() > inliersResult.size())
        {
            inliersResult = inliersCandidate;
        }
    }

    // Return indicies of inliers from fitted plane with most inliers

    return inliersResult;
}

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(const std::vector<PointT> &points, PointT point, int index, std::vector<int> &cluster, std::vector<int> &processed_points, KdTree *tree, float distanceTol)
{
    processed_points.push_back(index); // marking pt as processed
    cluster.push_back(index);
    auto nearby_points = tree->search(point, distanceTol);
    for (auto nearby_index : nearby_points)
    {
        if (!std::count(processed_points.begin(), processed_points.end(), nearby_index))
        { // has to be a better way of doing this
            // pt at that index not already processed
            proximity(points, points[nearby_index], nearby_index, cluster, processed_points, tree, distanceTol);
        }
    }
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<PointT> &points, KdTree *tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    std::vector<int> processed_points;
    int index = 0;
    while (index < points.size())
    {
        // check if processed
        if (std::count(processed_points.begin(), processed_points.end(), index))
        { // has to be a better way of doing this
            // pt at that index already processed
            index++;
            continue;
        }

        // new cluster
        std::vector<int> cluster;
        proximity(points, points[index], index, cluster, processed_points, tree, distanceTol);
        clusters.push_back(cluster);
        index++;
    }

    return clusters;
}
