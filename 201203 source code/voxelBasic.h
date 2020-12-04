#pragma once
#include <pcl/common/time.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
using namespace std;


// only key coordinate
struct voxelKey
{
	// coordinate
	int x;
	int y;
	int z;	
	// record iteration of exposure during 3D thinning
	int exposureIteration[5] = { 0,0,0,0,0 }; // [0] face exposure before 3D thinning (normally 0); [1-4] the latest iteration when not 1-4 face exposed
};


// light data package for voxel processing
struct voxelObject
{
	vector<voxelKey> objectSet;
	vector<voxelKey> voidSet;
	string fileName;
	pcl::PointXYZ originPoint;
	double resolution;
	voxelKey boundaryKey;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	int totalIteration;
};

// voxel skeleton branches
struct branch
{
	vector<voxelKey> branckKey;
	float voxelRadius;
	float ellipticalRatio;
};

void centroidToVoxelKey(pcl::PointCloud<pcl::PointXYZ>::VectorType& centroidPoints, double& resolution, pcl::PointXYZ& originPoint, vector<voxelKey>& centroindKey, voxelKey& boundaryKey);
void voxelKeyToCentroid(vector<voxelKey>& centroindKey, double& resolution, pcl::PointXYZ& originPoint, pcl::PointCloud<pcl::PointXYZ>::VectorType& centroidPoints);
