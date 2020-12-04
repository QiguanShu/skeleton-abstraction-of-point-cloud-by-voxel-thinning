#include "voxelBasic.h"
using namespace std;

class cloudVoxelization
{
public:
	// main function
	cloudVoxelization(string& fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inputCloud, double& resolution);
	// main data
	voxelObject voxelSet;

private:
	// subfunctions
	void cloudToOctree(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB>& voxelOctree);
	void octreeToCentroid(pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB>& voxelOctree, pcl::PointCloud<pcl::PointXYZ>::VectorType& centroidPoints, pcl::PointXYZ& originPoint);
};