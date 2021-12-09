#include <pcl/common/time.h>

#include "voxelProcessing.h"


/* processing the primary voxels
[in] voxelObject& voxelSet
[out] voxelObject& voxelSet
*/
voxelProcessing::voxelProcessing(voxelObject& voxelSet)
{
	// check if the voxel object has been processed
	if (!readVox(voxelSet.fileName, voxelSet.resolution, voxelSet))
	{
		// remove the noise
		deleteSeparate(voxelSet, 6);
		
		// set model boundary
		setBoundaryBox(voxelSet);

		// fill in the trunk hole	
		fillTrunkHole(voxelSet);

		// save the processed voxel data
		writeVox(voxelSet);
	}

	// rotate the model to test if different input rotation makes a diffenrence in 3D thinning
	//rotateModelinZAxis(voxelSet);

	// store processed data into a voxel model class
	finalizeVoxelObject(voxelSet);
}


/* remove separate noise from model's main body
[in] voxelObject& voxelSet
[in] const int type (connecting type 6, 18 or 26)
[out] voxelObject& voxelSet
*/
void voxelProcessing::deleteSeparate(voxelObject& voxelSet, const int type)
{
	cout << "\nremove separate voxels from the main body ... ";
	pcl::StopWatch timer;
	vector<voxelKey> deleteSet;
	for (voxelKey key : voxelSet.objectSet)
	{
		deleteSet.push_back(key);
	}
	// remove the noise
	removeSeparate(voxelSet.objectSet, type);
	setDifference(deleteSet, voxelSet.objectSet);
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << deleteSet.size() << " voxels are separated from main body and get removed" << endl;
	// view the deleted voxels
	color yellow;
	yellow.r = 1.0;
	yellow.g = 0.8;
	yellow.b = 0.2;
	color blue;
	blue.r = 0.2;
	blue.g = 0.8;
	blue.b = 1.0;
	voxelViewer viewRemove(voxelSet);
	viewRemove.addVoxelKey(voxelSet.objectSet, 1, blue, false, "objectSet");
	viewRemove.addVoxelKey(deleteSet, 1, yellow, true, "deleteSet");
	viewRemove.run();
}


/* set a boundary box for the model
[in] voxelObject& voxelSet
[out] voxelObject& voxelSet
*/
void voxelProcessing::setBoundaryBox(voxelObject& voxelSet)
{
	cout << "\nset the boundary box (all voxel space including object and background voxels) according to user input ..." << endl;
	cout << "current voxels range is from (0, 0, 0) to (" << voxelSet.boundaryKey.x << ", " << voxelSet.boundaryKey.y << ", " << voxelSet.boundaryKey.z << ")" << endl;
	cout << "please input the boundary box in the rendered viewing window ..." << endl;
	voxelKey minBoundary, maxBoundary;
	minBoundary.x = 0;
	minBoundary.y = 0;
	minBoundary.z = 0;
	maxBoundary = voxelSet.boundaryKey;
	// viewing window to store input in minBoundary and maxBoundary
	inputBoundaryBox inputWindow(voxelSet, minBoundary,maxBoundary);
	// move the whole voxel object to make the minimum key at (0,0,0)
	vector<voxelKey> newObjectSet;
	for(voxelKey key : voxelSet.objectSet)
	{
		if (minBoundary.x <= key.x && key.x <= maxBoundary.x &&
			minBoundary.y <= key.y && key.y <= maxBoundary.y &&
			minBoundary.z <= key.z && key.z <= maxBoundary.z)
		{
		voxelKey newKey;
		newKey.x = key.x - minBoundary.x;
		newKey.y = key.y - minBoundary.y;
		newKey.z = key.z - minBoundary.z;
		newObjectSet.push_back(newKey);
		}
	}
	voxelSet.objectSet.clear();
	for(voxelKey key : newObjectSet)
	{
		voxelSet.objectSet.push_back(key);
	}
	voxelSet.boundaryKey.x = maxBoundary.x - minBoundary.x;
	voxelSet.boundaryKey.y = maxBoundary.y - minBoundary.y;
	voxelSet.boundaryKey.z = maxBoundary.z - minBoundary.z;
	voxelSet.originPoint.x += minBoundary.x * voxelSet.resolution;
	voxelSet.originPoint.y += minBoundary.y * voxelSet.resolution;
	voxelSet.originPoint.z += minBoundary.z * voxelSet.resolution;
	cout << "model new range is from (0, 0, 0) to (" << voxelSet.boundaryKey.x << ", " << voxelSet.boundaryKey.y << ", " << voxelSet.boundaryKey.z << ")" << endl;
	cout << "total object voxels within the new range: " << voxelSet.objectSet.size() << endl;
}


/* fill in trunk holes by continuity of void voxels
[in] voxelObject& voxelSet
[out] voxelObject& voxelSet
*/
void voxelProcessing::fillTrunkHole(voxelObject& voxelSet)
{
	// input object voxels to solve the surface flaw
	cout << "\nplease input object voxels to seal small holes on the trunk surface ..." << endl;
	vector<voxelKey> inputSet;
	inputVoxelKey(voxelSet,inputSet);
	for(voxelKey key : inputSet)
	{
		voxelSet.objectSet.push_back(key);
	}
	// input one void voxel at the outside
	cout << "\nplease input at least one void voxel at outside to start the continuity calculation ..." << endl;
	vector<voxelKey> outsideVoidSet;
	do
	{
		inputVoxelKey(voxelSet, outsideVoidSet);
	} while (outsideVoidSet.size() == 0);
	// fill the trunk hole
	cout << "\nfill in void voxels inside the trunks ...";
	pcl::StopWatch timer;
	vector<voxelKey> uniSet;
	universalSet(voxelSet.boundaryKey, uniSet);
	vector<voxelKey> allVoidSet;
	int oldSize = voxelSet.objectSet.size();
	relativeComplement(uniSet, voxelSet.objectSet, allVoidSet);
	getConnected(outsideVoidSet[0], allVoidSet, 6, voxelSet.voidSet);
	relativeComplement(uniSet, voxelSet.voidSet, voxelSet.objectSet);
	cout << "taking " << timer.getTimeSeconds() << " seconds... completd" << endl;
	cout << "fill in " << voxelSet.objectSet.size() - oldSize << " object voxels inside the trunk" << endl;
	cout << "final object voxels: " << voxelSet.objectSet.size() << endl;
	cout << "final void voxels: " <<voxelSet.voidSet.size() << endl;
}


/* finalize voxel object for 3D thinning
[in] voxelObject& voxelSet
*/
void voxelProcessing::finalizeVoxelObject(voxelObject& voxelSet)
{
	cout << "\nfinalize voxel object class for 3D thinning ...";
	pcl::StopWatch timer;
	if(voxelSet.voidSet.size() == 0) // it means the voxel object is read from .vox file
	{
		vector<voxelKey> uniSet;
		universalSet(voxelSet.boundaryKey, uniSet);
		relativeComplement(uniSet, voxelSet.objectSet, voxelSet.voidSet);
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;

	// view the model
	cout << "\nview final voxel model before 3D thinning ..." << endl;
	voxelViewer viewVoxel(voxelSet);
	color blue;
	blue.r = 0.2;
	blue.g = 0.8;
	blue.b = 1.0;
	viewVoxel.addVoxelKey(voxelSet.objectSet, 1, blue, false, "objectSet");
	viewVoxel.run();
	cout << endl;
}


/* rotate the whole model in z axis 90 degree
[in] voxelObject& voxelSet
[out] voxelObject& voxelSet
*/
void voxelProcessing::rotateModelinZAxis(voxelObject& voxelSet)
{
	cout << "\nrotate the model 90 degrees in z axis ...";
	pcl::StopWatch timer;
	// change file name
	voxelSet.fileName.insert(voxelSet.fileName.find(".ply"), "_rotated");
	// change file name
	voxelKey newBoundaryKey;
	newBoundaryKey.x = voxelSet.boundaryKey.y;
	newBoundaryKey.y = voxelSet.boundaryKey.x;
	newBoundaryKey.z = voxelSet.boundaryKey.z;
	voxelSet.boundaryKey = newBoundaryKey;
	// change objectSet
	vector<voxelKey> newObjectSet;
	for(voxelKey key : voxelSet.objectSet)
	{
		voxelKey newKey;
		newKey.x = -key.y + voxelSet.boundaryKey.x;
		newKey.y = key.x;
		newKey.z = key.z;
		newObjectSet.push_back(newKey);
	}
	voxelSet.objectSet.clear();
	for(voxelKey key : newObjectSet)
	{
		voxelSet.objectSet.push_back(key);
	}
	// change voidSet
	if(voxelSet.voidSet.size() != 0)
	{
		vector<voxelKey> newVoidSet;
		for (voxelKey key : voxelSet.voidSet)
		{
			voxelKey newKey;
			newKey.x = -key.y + voxelSet.boundaryKey.x;
			newKey.y = key.x;
			newKey.z = key.z;
			newVoidSet.push_back(newKey);
		}
		voxelSet.voidSet.clear();
		for (voxelKey key : newVoidSet)
		{
			voxelSet.voidSet.push_back(key);
		}
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << "file name: " << voxelSet.fileName << endl;
	cout << "boundaryKey (" << voxelSet.boundaryKey.x << ", " << voxelSet.boundaryKey.y << ", " << voxelSet.boundaryKey.z << ")" << endl;
}
