#include<iostream>

#include "cloudInitialization.h"
#include "cloudVoxelization.h"
#include "voxelProcessing.h"
#include "voxel3DThinning.h"
#include "skeletonAnalysis.h"

using namespace std;


int main()
{	
	string fileName;
	cout << "---------------------------------------- user input ----------------------------------------" << endl;
	cout << "Please enter the name of .ply file to load (e.g. living_bridge.ply): ";
	cin >> fileName;

	// initialize the point cloud data
	cloudInitialization initializedCloud(fileName);

	double resolution;
	cout << "---------------------------------------- user input ----------------------------------------" << endl;
	cout << "Please enter the resolution for voxels (e.g. 0.01): ";
	cin >> resolution;

	// voxelize cloud and generate voxel keys
	cloudVoxelization voxelizedCloud(initializedCloud.fileName, initializedCloud.cloud, resolution);

	// process the voxel set
	voxelProcessing processedVoxel(voxelizedCloud.voxelSet);

	// 3D thinning
	voxel3DThinning voxelSkeleton(voxelizedCloud.voxelSet);

	// reconstruct the model from skeleton
	skeletonAnalysis rebuiltModel(voxelSkeleton.voxelSet, voxelizedCloud.voxelSet);

	return 0;
}