#include "fileManager.h"
using namespace std;

class cloudInitialization
{
public:
	// main function
	cloudInitialization(string name);
	// main data
	string fileName;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	// further model information
	pcl::PointXYZRGB minPoint, maxPoint;
	double cloudLength, cloudDepth, cloudHeight;

private:
	// subfunctions
	void calculatePCA(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector4f& centroid, Eigen::Matrix3f& eigDx);
	void transformCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector4f& centroid, Eigen::Matrix3f& eigDx);
	void getCloudInformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointXYZRGB& minPoint, pcl::PointXYZRGB& maxPoint, double& cloudLength, double& cloudDepth, double& cloudHeight);
};