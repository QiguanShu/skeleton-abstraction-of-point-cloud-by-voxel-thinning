#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <eigen3/Eigen/Dense>

#include "cloudInitialization.h"


/* initialize .ply file containing a point cloud
[in] string name
[result] get an object of this class
*/
cloudInitialization::cloudInitialization(string name):
	fileName(name),
	cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	// read .ply file
	if (!readPly(fileName, cloud))
	{
		cout << "\nloading .ply file error" << endl;
	}
	else
	{
		// compute principal direction
		Eigen::Vector4f centroid;
		Eigen::Matrix3f eigDx;
		calculatePCA(cloud, centroid, eigDx);

		// transformation
		transformCloud(cloud, centroid, eigDx);

		// save the initialized cloud
		string initializedFileName = fileName;
		initializedFileName.insert(fileName.length() - 4, "_initialized");
		writePly(initializedFileName, cloud);
	}
	
	// show the basic model information
	getCloudInformation(cloud, minPoint, maxPoint, cloudLength, cloudDepth, cloudHeight);
	cout << "the original cloud has been initialized\n" << endl;
}



/* calculate the principal component eigenvector of the PointXYZRGB cloud
[in] pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
[out] Eigen::Vector4f& centroid
[out] Eigen::Matrix3f& eigDx
*/
void cloudInitialization::calculatePCA(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector4f& centroid, Eigen::Matrix3f& eigDx)
{
	cout << "\ncalculating principal component eigenvector ... ";
	pcl::StopWatch timer;
	pcl::compute3DCentroid(*cloud, centroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	eigDx = eigen_solver.eigenvectors();
	eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1)); // This line is necessary for proper orientation in some cases. The numbers come out the same without it, but the signs are different and the box doesn't get correctly oriented in some cases. 
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << "EigenVectors: " << endl << eigDx << endl;
	cout << "Eigenvalues: " << endl << eigen_solver.eigenvalues() << endl;
}


/* orientate the original cloud according to principal direction
[in] pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
[in] Eigen::Vector4f& centroid
[in] Eigen::Matrix3f& eigDx
[out] pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
*/
void cloudInitialization::transformCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector4f& centroid, Eigen::Matrix3f& eigDx)
{
	cout << "\ntransforming position and orientation ... ";
	pcl::StopWatch timer;
	Eigen::Matrix4f transformMatrix(Eigen::Matrix4f::Identity());
	transformMatrix.block<3, 3>(0, 0) = eigDx.transpose();
	transformMatrix.block<3, 1>(0, 3) = -1.f * (transformMatrix.block<3, 3>(0, 0) * centroid.head<3>());
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << "Transform Matrix: " << endl << transformMatrix << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr initializedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*cloud, *initializedCloud, transformMatrix);
	cloud = initializedCloud;
}


/* save basic information of the transformed cloud to this object
[in] pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
[out] pcl::PointXYZRGB& minPoint
[out] pcl::PointXYZRGB& maxPoint
[out] double& cloudLength
[out] double& cloudDepth
[out] double& cloudHeight
*/
void cloudInitialization::getCloudInformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointXYZRGB& minPoint, pcl::PointXYZRGB& maxPoint, double& cloudLength, double& cloudDepth, double& cloudHeight)
{
	pcl::getMinMax3D(*cloud, minPoint, maxPoint);	
	cloudLength = maxPoint.x - minPoint.x;
	cloudDepth = maxPoint.y - minPoint.y;
	cloudHeight = maxPoint.z - minPoint.z;
	cout << "\n" << fileName << " model information overview" << endl;
	cout << "minimum point: " << minPoint << endl;
	cout << "maximum point: " << maxPoint << endl;
	cout << "model length: " << cloudLength << endl;
	cout << "model depth: " << cloudDepth << endl;
	cout << "model height: " << cloudHeight << endl;
}
