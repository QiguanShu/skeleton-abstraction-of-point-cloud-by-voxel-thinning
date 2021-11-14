#include "cloudVoxelization.h"


/* voxelize point cloud
[in] string fileName
[in] pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inputCloud
[in] double resolution
[result] get an object of this class
*/
cloudVoxelization::cloudVoxelization(string& fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inputCloud, double& resolution)
{
	// structure point cloud as octree
	pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB> voxelOctree(resolution);
	cloudToOctree(inputCloud, voxelOctree);

	// calculate centroid points of all voxels
	pcl::PointCloud<pcl::PointXYZ>::VectorType centroidPoints;
	octreeToCentroid(voxelOctree, centroidPoints, voxelSet.originPoint);

	// generate voxel keys from centroid point coordinats
	centroidToVoxelKey(centroidPoints, resolution, voxelSet.originPoint, voxelSet.objectSet, voxelSet.boundaryKey);

	// store other information in the voxel object
	voxelSet.fileName = fileName;
	voxelSet.resolution = resolution;
	voxelSet.cloud = inputCloud;
}


/* set cloud to the octree
[in] pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
[out] pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB>& voxelOctree
*/
void cloudVoxelization::cloudToOctree(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB>& voxelOctree)
{
	cout << "\nstructring point cloud as an octree ...";
	pcl::StopWatch timer;
	voxelOctree.setInputCloud(cloud);
	voxelOctree.defineBoundingBox();
	voxelOctree.addPointsFromInputCloud();
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << "total octree depth: " << voxelOctree.getTreeDepth() << endl;
	cout << "total octree leaves: " << voxelOctree.getLeafCount() << endl;
}


/* get centroid points of each octree leaf
[in] pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB>& voxelOctree
[out] pcl::PointCloud<pcl::PointXYZ>::VectorType& centroidPoints
[out] pcl::PointXYZ& originPoint
*/
void cloudVoxelization::octreeToCentroid(pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB>& voxelOctree, pcl::PointCloud<pcl::PointXYZ>::VectorType& centroidPoints, pcl::PointXYZ& originPoint)
{
	cout << "\nextracting centroid points of every octree leaves ...";
	pcl::StopWatch timer;
	int depth = static_cast<int> (floor(voxelOctree.getTreeDepth()));
	pcl::PointXYZ pt_leaf_center;
	// for each octree node at the biggest depth (octree leaf)
	for (pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB>::FixedDepthIterator tree_iter = voxelOctree.fixed_depth_begin(depth);	tree_iter != voxelOctree.fixed_depth_end();	++tree_iter)
	{
		// calculate the center of the each octree leaf space
		Eigen::Vector3f leaf_min, leaf_max;
		voxelOctree.getVoxelBounds(tree_iter, leaf_min, leaf_max);
		pt_leaf_center.x = (leaf_min.x() + leaf_max.x()) / 2.0f;
		pt_leaf_center.y = (leaf_min.y() + leaf_max.y()) / 2.0f;
		pt_leaf_center.z = (leaf_min.z() + leaf_max.z()) / 2.0f;
		centroidPoints.push_back(pt_leaf_center);

		// set the origin point to be the minimum centroid point -1 in x,y,z axis
		if (pt_leaf_center.x < originPoint.x + voxelOctree.getResolution() / (float)2)
		{
			originPoint.x = pt_leaf_center.x - voxelOctree.getResolution();
		}
		if (pt_leaf_center.y < originPoint.y + voxelOctree.getResolution() / (float)2)
		{
			originPoint.y = pt_leaf_center.y - voxelOctree.getResolution();
		}
		if (pt_leaf_center.z < originPoint.z + voxelOctree.getResolution() / (float)2)
		{
			originPoint.z = pt_leaf_center.z - voxelOctree.getResolution();
		}
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << "total centroid points: " << centroidPoints.size() << endl;
	cout << "origin point: " << originPoint << endl;
}
