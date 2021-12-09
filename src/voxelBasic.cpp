#include "voxelBasic.h"


/* interpret coordinates of centroid points to voxel keys
[in] pcl::PointCloud<pcl::PointXYZ>::VectorType& centroidPoints
[in] double& resolution
[in]  pcl::PointXYZ& originPoint
[out] vector<voxelKey>& centroindKey
[out] voxelKey& boundaryKey
*/
void centroidToVoxelKey(pcl::PointCloud<pcl::PointXYZ>::VectorType& centroidPoints, double& resolution, pcl::PointXYZ& originPoint, vector<voxelKey>& centroindKey, voxelKey& boundaryKey)
{
	cout << "\ninterpreting coordinates of centroid point to voxel keys ...";
	pcl::StopWatch timer;
	// clear the previous data
	centroindKey.clear();
	boundaryKey.x = 0;
	boundaryKey.y = 0;
	boundaryKey.z = 0;
	// for each centroid point
	for (pcl::PointXYZ point : centroidPoints)
	{
		voxelKey key;
		key.x = (point.x - originPoint.x + resolution / 10) / resolution;
		key.y = (point.y - originPoint.y + resolution / 10) / resolution;
		key.z = (point.z - originPoint.z + resolution / 10) / resolution;
		centroindKey.push_back(key);
		// get the maximum voxel key + 1 in x,y,z axis as the boundaryKey
		if (key.x >= boundaryKey.x)
		{
			boundaryKey.x = key.x + 1;
		}
		if (key.y >= boundaryKey.y)
		{
			boundaryKey.y = key.y + 1;
		}
		if (key.z >= boundaryKey.z)
		{
			boundaryKey.z = key.z + 1;
		}
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << centroindKey.size() << " voxel keys have been generated" << endl;
	cout << "boundary key: (" << boundaryKey.x << ", " << boundaryKey.y << ", " << boundaryKey.z << ")" << endl;
}


/* interpret voxel keys to coordinates of centroid points
[in] vector<voxelKey>& centroindKey
[in] double& resolution
[in] pcl::PointXYZ& originPoint
[out] pcl::PointCloud<pcl::PointXYZ>::VectorType& centroidPoints
*/
void voxelKeyToCentroid(vector<voxelKey>& centroindKey, double& resolution, pcl::PointXYZ& originPoint, pcl::PointCloud<pcl::PointXYZ>::VectorType& centroidPoints)
{
	// clear the previous data
	centroidPoints.clear();
	// for each voxel key
	for (voxelKey key : centroindKey)
	{
		pcl::PointXYZ point;
		point.x = key.x * resolution + originPoint.x;
		point.y = key.y * resolution + originPoint.y;
		point.z = key.z * resolution + originPoint.z;
		centroidPoints.push_back(point);
	}
}
