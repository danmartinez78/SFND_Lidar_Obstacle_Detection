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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	// TODO: Fill in this function
	srand(time(NULL));
	int numPoints = cloud->points.size();
	std::srand(std::time(nullptr));
	// For max iterations
	for (int i = 0;i<maxIterations;i++){
		// Randomly sample subset and fit line
		int index_1 = (rand() % numPoints) + 1;
		int index_2 = (rand() % numPoints) + 1;
		int index_3 = (rand() % numPoints) + 1;
		while (index_1 == index_2 || index_2 == index_3 || index_3 == index_1){
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
		float D = -(A*pt1.x + B*pt1.y + C*pt1.z);

		
		float A_squared = std::pow(A,2);
		float B_squared = std::pow(B,2);
		float C_squared = std::pow(C,2);
		float denominator = std::sqrt(A_squared + B_squared + C_squared);

		// Measure distance between every point and fitted line
		std::unordered_set<int> inliersCandidate;
		for (int j = 0; j<numPoints; j++){
			auto candidatePt = cloud->points[j];
			float distance = std::abs(A*candidatePt.x + B*candidatePt.y + C*candidatePt.z + D) / denominator;
			if (distance <= distanceTol){
				// If distance is smaller than threshold count it as inlier
				inliersCandidate.insert(j);
			}
		}
		if(inliersCandidate.size()>inliersResult.size()){
			inliersResult = inliersCandidate;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, .5);

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
