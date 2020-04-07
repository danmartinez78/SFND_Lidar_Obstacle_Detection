// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>

template <typename PointT>
class ProcessPointClouds
{
public:
    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    struct Node
    {
        PointT point;
        int id;
        Node *left;
        Node *right;

        Node(PointT arr, int setId)
            : point(arr), id(setId), left(NULL), right(NULL)
        {
        }
    };

    struct KdTree
    {
        Node *root;

        KdTree()
            : root(NULL)
        {
        }

        void insert(PointT point, int id)
        {
            // the function should create a new node and place correctly with in the root
            recurssive_insert(&root, 0, point, id);
        }

        void recurssive_insert(Node **node, int depth, PointT data, int id)
        {
            if (*node == NULL)
            {
                *node = new Node(data, id);
            }
            else
            {
                int split_dem = depth % 2; // 2D Tree
                Node **new_node;
                if (data[split_dem] >= ((*node)->point[split_dem]))
                {
                    // greatear than => split right
                    new_node = &((*node)->right);
                }
                else
                {
                    // less than => split left
                    new_node = &((*node)->left);
                }
                // reduction step
                recurssive_insert(new_node, depth + 1, data, id);
            }
        }

        // return a list of point ids in the tree that are within distance of target
        std::vector<int> search(PointT target, float distanceTol)
        {
            std::vector<int> ids;
            // start at root
            recurssive_search(root, target, distanceTol, 0, ids);
            return ids;
        }

        void recurssive_search(Node *node, PointT target, float distanceTol, int depth, std::vector<int> &ids)
        {
            int split_dem = depth % 2; // 2D Tree
            if (node != NULL)
            {
                // check if passed node is within tol along both directions
                float d[2] = {(target[0] - node->point[0]), (target[1] - node->point[1])};
                if (std::fabs(d[0]) <= distanceTol && std::fabs(d[1]) <= distanceTol)
                {
                    if (std::sqrt(std::pow(d[0], 2) + std::pow(d[1], 2)) <= distanceTol)
                    {
                        ids.push_back(node->id);
                    }
                }
                if (d[split_dem] < distanceTol)
                {
                    recurssive_search(node->left, target, distanceTol, depth + 1, ids);
                }
                if (d[split_dem] > -distanceTol)
                {
                    recurssive_search(node->right, target, distanceTol, depth + 1, ids);
                }
            }
        }
    };

    KdTree *tree;

    void proximity(const std::vector<PointT> &points, PointT point, int index, std::vector<int> &cluster, std::vector<int> &processed_points, KdTree *tree, float distanceTol);

    std::vector<std::vector<int>> euclideanCluster(const std::vector<PointT> &points, KdTree *tree, float distanceTol);
};
#endif /* PROCESSPOINTCLOUDS_H_ */