#include "skeletonAnalysis.h"
#include "fileManager.h"


/* analyze the voxel skeleton
[in] voxelObject& skeletonSet
[in] voxelObject& voxelModelSet
*/
skeletonAnalysis::skeletonAnalysis(voxelObject& skeletonSet, voxelObject& voxelModelSet)
{
	// separate branches
	vector<branch> branches;
	separateBranch(skeletonSet.objectSet, branches);

	// write branches to .bra file
	writeBra(skeletonSet, branches);

	// reconstruct the model from skeleton information
	rebuildModel(skeletonSet, 2, voxelModelSet.objectSet);

	/* use branch's elliptical ration to revise
	rebuildModel(skeletonSet, 2, branches, voxelModelSet.objectSet);
	*/
}


/* estimate the elliptical ration of an input branch voxelKey set using mean 2-:1-face iteration of exposure
[in] voxelObject& skeletonSet
[out] float& ellipticalRatio
*/
void skeletonAnalysis::estimateEllipticalRatio(vector<voxelKey>& skeletonKey, float& ellipticalRatio)
{
	float mean1Face = 0;
	float mean2Face = 0;
	for(voxelKey key : skeletonKey)
	{
		mean1Face += key.exposureIteration[1];
		mean2Face += key.exposureIteration[2];
	}
	mean1Face = mean1Face / skeletonKey.size();
	mean2Face = mean2Face / skeletonKey.size();
	ellipticalRatio = ((float)mean2Face / (float)mean1Face + 0.485) / 1.1916;
}


/* estimate the average section radius of an input branch voxelKey set
[in] voxelObject& skeletonSet
[out] float& ellipticalRatio
*/
void skeletonAnalysis::estimateRadius(vector<voxelKey>& skeletonKey, const int type, float& voxelRadius)
{
	voxelRadius = 0;
	for (voxelKey key : skeletonKey)
	{
		voxelRadius += key.exposureIteration[type];
	}
	voxelRadius = (float)voxelRadius / skeletonKey.size();
}


/* expand key in x and y axis according the face exposure iteration
[in] voxelKey& skeletonKey
[in] const int type (type = 1, 2, 3, 4; use 1-, 2-, 3- or 4- face exposure iteration to expand the skeleton)
[result] save expand voxel in the voxelSet
*/
void skeletonAnalysis::expandSkeletonKey(voxelKey& skeletonKey, const int type)
{
	for(int x = skeletonKey.x - skeletonKey.exposureIteration[4]; x < skeletonKey.x + skeletonKey.exposureIteration[4]; x++)
	{
		for (int y = skeletonKey.y - skeletonKey.exposureIteration[4]; y < skeletonKey.y + skeletonKey.exposureIteration[4]; y++)
		{
			// check if voxel is within the radius
			if(pow(x - skeletonKey.x, 2) + pow(y - skeletonKey.y, 2) <= pow(skeletonKey.exposureIteration[type], 2))
			{
				voxelKey key;
				key.x = x;
				key.y = y;
				key.z = skeletonKey.z;
				if(!belongsToSet(key, rebuiltModel.objectSet))
				{
					rebuiltModel.objectSet.push_back(key);
				}
			}
		}
	}
}


/* expand a key in x and y axis according the face exposure iteration and elliptical ratio
[in] voxelKey& skeletonKey
[in] const int type (type = 1, 2, 3, 4; use 1-, 2-, 3- or 4- face exposure iteration to expand the skeleton)
[in] float& ellipticalRatio
[result] save expand voxel in the voxelSet
*/
void skeletonAnalysis::expandSkeletonKey(voxelKey& skeletonKey, const int type, float& ellipticalRatio)
{
	for (int x = skeletonKey.x - skeletonKey.exposureIteration[4]; x < skeletonKey.x + skeletonKey.exposureIteration[4]; x++)
	{
		for (int y = skeletonKey.y - skeletonKey.exposureIteration[4]; y < skeletonKey.y + skeletonKey.exposureIteration[4]; y++)
		{
			// check if voxel is within the radius of the 4-face exposure
			if(type == 1 || type == 2)
			{
				if (pow(x - skeletonKey.x, 2) + pow(y - skeletonKey.y, 2) <= pow((skeletonKey.exposureIteration[type]) * sqrt(ellipticalRatio), 2))
				{
					voxelKey key;
					key.x = x;
					key.y = y;
					key.z = skeletonKey.z;
					if (!belongsToSet(key, rebuiltModel.objectSet))
					{
						rebuiltModel.objectSet.push_back(key);
					}
				}
			}
			if (type == 3 || type == 4)
			{
				if (pow(x - skeletonKey.x, 2) + pow(y - skeletonKey.y, 2) <= pow((skeletonKey.exposureIteration[type]) / sqrt(ellipticalRatio), 2))
				{
					voxelKey key;
					key.x = x;
					key.y = y;
					key.z = skeletonKey.z;
					if (!belongsToSet(key, rebuiltModel.objectSet))
					{
						rebuiltModel.objectSet.push_back(key);
					}
				}
			}
		}
	}
}


/* reconstruct the model out of voxel skeleton
[in] voxelObject& skeletonSet
[in] const int type (type = 1, 2, 3, 4; use 1-, 2-, 3- or 4- face exposure iteration to expand the skeleton)
[in] vector<voxelKey>& voxelModelSet
[result] save trunk model in the voxelSet
*/
void skeletonAnalysis::rebuildModel(voxelObject& skeletonSet, const int type, vector<voxelKey>& voxelModelSet)
{
	// initialize basic information
	rebuiltModel.fileName = skeletonSet.fileName;
	rebuiltModel.resolution = skeletonSet.resolution;
	rebuiltModel.originPoint = skeletonSet.originPoint;
	rebuiltModel.boundaryKey = skeletonSet.boundaryKey;
	rebuiltModel.cloud = skeletonSet.cloud;
	rebuiltModel.totalIteration = skeletonSet.totalIteration;
	rebuiltModel.objectSet.clear();

	// expand each skeleton key
	cout << "\nexpand skeleton to reconstruct the model using the " << type << " face exposure iteration ...";
	pcl::StopWatch timer;
	for (voxelKey key : skeletonSet.objectSet)
	{
		expandSkeletonKey(key, type);
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << "total expanded voxels: " << rebuiltModel.objectSet.size() << endl;
	
	// compare the rebuilt model with origin model
	modelCompare(voxelModelSet, rebuiltModel.objectSet);

	// save rebuild model to .reb
	writeReb(rebuiltModel, commonSet, missingSet, redundantSet);

	// view the result
	///*
	voxelViewer viewRebuild(rebuiltModel);
	color yellow;
	yellow.r = 1.0;
	yellow.g = 0.8;
	yellow.b = 0.2;
	viewRebuild.addVoxelKey(rebuiltModel.objectSet, 1, yellow, false, "objectSet");
	viewRebuild.run();
	//*/
}


/* reconstruct the model out of voxel skeleton based on branches
[in] voxelObject& skeletonSet
[in] const int type (type = 1, 2, 3, 4; use 1-, 2-, 3- or 4- face exposure iteration to expand the skeleton)
[in] vector<voxelKey>& voxelModelSet
[result] save trunk model in the voxelSet
*/
void skeletonAnalysis::rebuildModel(voxelObject& skeletonSet, const int type, vector<branch>branches, vector<voxelKey>& voxelModelSet)
{
	// initialize basic information
	rebuiltModel.fileName = skeletonSet.fileName;
	rebuiltModel.resolution = skeletonSet.resolution;
	rebuiltModel.originPoint = skeletonSet.originPoint;
	rebuiltModel.boundaryKey = skeletonSet.boundaryKey;
	rebuiltModel.cloud = skeletonSet.cloud;
	rebuiltModel.totalIteration = skeletonSet.totalIteration;
	rebuiltModel.objectSet.clear();

	// expand each skeleton key
	cout << "\nexpand skeleton to reconstruct the model using the " << type << " face exposure iteration ...";
	pcl::StopWatch timer;
	for (branch oneBranch: branches)
	{
		for(voxelKey key : oneBranch.branckKey)
		{
			expandSkeletonKey(key, type, oneBranch.ellipticalRatio);
		}
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << "total expanded voxels: " << rebuiltModel.objectSet.size() << endl;

	// compare the rebuilt model with origin model
	modelCompare(voxelModelSet, rebuiltModel.objectSet);

	// save rebuild model to .reb
	writeReb(rebuiltModel, commonSet, missingSet, redundantSet);

	// view the result
	///*
	voxelViewer viewRebuild(rebuiltModel);
	color yellow;
	yellow.r = 1.0;
	yellow.g = 0.8;
	yellow.b = 0.2;
	viewRebuild.addVoxelKey(rebuiltModel.objectSet, 1, yellow, false, "objectSet");
	viewRebuild.run();
	//*/
}


/* compare rebuilt model and origin voxel model
[in] voxelObject& voxelModelSet
[in] voxelObject& rebuiltModel
[result] print information in the console
*/
void skeletonAnalysis::modelCompare(vector<voxelKey>& voxelModelSet, vector<voxelKey>& rebuiltModel)
{
	for(voxelKey key : voxelModelSet)
	{
		if(belongsToSet(key, rebuiltModel))
		{
			commonSet.push_back(key);
		}
		else 
		{
			missingSet.push_back(key);
		}
	}	
	for (voxelKey key : rebuiltModel)
	{
		if (!belongsToSet(key, voxelModelSet))
		{
			redundantSet.push_back(key);
		}
	}
	cout << "total origin voxels: " << voxelModelSet.size() << endl;
	printf("common voxels: %d  (%.2f%% of origin model / %.2f%% of rebuilt model)\n", commonSet.size(), (float)commonSet.size() / (float)voxelModelSet.size() * 100, (float)commonSet.size() / (float)rebuiltModel.size() * 100);
	printf("redundant voxels: %d  (%.2f%% of origin model / %.2f%% of rebuilt model)\n", redundantSet.size(), (float)redundantSet.size() / (float)voxelModelSet.size() * 100, (float)redundantSet.size() / (float)rebuiltModel.size() * 100);
	printf("missing voxels: %d  (%.2f%% of origin model / %.2f%% of rebuilt model)\n", missingSet.size(), (float)missingSet.size() / (float)voxelModelSet.size() * 100, (float)missingSet.size() / (float)rebuiltModel.size() * 100);
}


/* calculate a true joint point in a joint group
[in] vector<voxelKey>& jointGroup
[out] voxelKey& newJoint
*/
void skeletonAnalysis::findTrueJoint(vector<voxelKey>& jointGroup, voxelKey& newJoint)
{
	if (jointGroup.size() > 1)
	{
		// one of the least common connected voxel in the joint group is regarded as true joint point
		int minConnection = 26;
		for(voxelKey key : jointGroup)
		{
			int connection = getTargetSurroundings(key, jointGroup).allNeighborSet.size();
			if(connection < minConnection)
			{
				newJoint = key;
				minConnection = connection;
			}
		}
	}
	else
	{
		newJoint = jointGroup[0];
	}
	// delete voxels that are not adjacent to true joint neighbor from jointGroup
	vector<voxelKey> trueJointNeighbor = getTargetSurroundings(newJoint, jointGroup).allNeighborSet;
	trueJointNeighbor.push_back(newJoint);
	if(trueJointNeighbor.size() < jointGroup.size())
	{
		for (voxelKey key : jointGroup)
		{
			if (!belongsToSet(key, trueJointNeighbor))
			{
				setDifference(jointGroup, key);
			}
		}
	}
}


/* pick out all joint points in the skeleton
[in] vector<voxelKey>& skeletonSet
[out] vector<jointpoint>& jointpoints
[out] vector<voxelKey>& trueJoint
*/
void skeletonAnalysis::findJointpoints(vector<voxelKey>& skeletonSet, vector<voxelKey>& jointpoints, vector<voxelKey>& trueJoint)
{
	// get all joint points that have more than 2 neighbor voxels in the skeleton set
	for(voxelKey key : skeletonSet)
	{
		getTargetSurroundings skeletonNeighbor(key, skeletonSet);
		if(skeletonNeighbor.allNeighborSet.size() > 2)
		{
			jointpoints.push_back(key);
		}
	}
	// if joint points are found
	if(jointpoints.size() > 0)
	{
		// check all joint points if they are adjacent to each other as small groups
		vector<voxelKey> uncheckedJoint;
		uncheckedJoint = jointpoints;
		do
		{
			voxelKey inspectJoint;
			for (voxelKey key : jointpoints)
			{
				if (belongsToSet(key, uncheckedJoint))
				{
					inspectJoint = key;
					break;
				}
			}
			vector<voxelKey> jointGroup;
			getConnected(inspectJoint, uncheckedJoint, 26, jointGroup);
			voxelKey newJoint;
			findTrueJoint(jointGroup, newJoint);
			trueJoint.push_back(newJoint);
			setDifference(uncheckedJoint, jointGroup);
		} while (uncheckedJoint.size() > 0);
	}
}


/* track down the the skeleton from a joint point to get one branch
[in] voxelKey& startJoint
[in] vector<voxelKey>& jointpoints
[in] vector<voxelKey>& trueJoint
[in] vector<voxelKey>& skeletonSet
[out] vector<voxelKey>& branchSet
*/
void skeletonAnalysis::getOneBranch(voxelKey& start, vector<voxelKey>& jointpoints, vector<voxelKey>& trueJoint, vector<voxelKey>& skeletonSet, vector<voxelKey>& branchSet)
{
	branchSet.clear();
	branchSet.push_back(start);
	voxelKey inspectKey = start;
	bool branchStart, branchEnd;
	branchEnd = false;
	do 
	{
		bool normalNeighbor = false;
		bool trueJointNeighbor = false;
		getTargetSurroundings neighborSkeleton(inspectKey, skeletonSet);
		// tell if the inspect key comes to an branch end that is not at any joint point
		if(neighborSkeleton.allNeighborSet.size() > 0)
		{
			for (voxelKey key : neighborSkeleton.allNeighborSet)
			{
				// normal voxels (voxels that don't belong to joint set) have the priority to be identified as a branch
				if (!belongsToSet(key, jointpoints))
				{
					inspectKey = key;
					normalNeighbor = true;
					break;
				}
			}
			if (normalNeighbor) // normal neighbor voxel is found
			{
				branchSet.push_back(inspectKey);
				setDifference(skeletonSet, inspectKey);
			}
			else // only have joint point as neighbor
			{
				// trueJoints that are not the branch start have the secondary priority to be identified as a branch end
				for (voxelKey key : neighborSkeleton.allNeighborSet)
				{
					if (belongsToSet(key, trueJoint) && (key.x != start.x || key.y != start.y || key.z != start.z))
					{
						trueJointNeighbor = true;
						branchEnd = true;
						branchSet.push_back(key);
						break;
					}
				}
				if (!trueJointNeighbor)
				{
					bool findNextJoint = false;
					for (voxelKey key : neighborSkeleton.allNeighborSet)
					{
						// voxels not belong to true joint have the secondary priority to be identified as a branch
						if (!belongsToSet(key, trueJoint))
						{
							inspectKey = key;
							branchSet.push_back(inspectKey);
							setDifference(skeletonSet, inspectKey);
							findNextJoint = true;
							break;
						}
					}
					if(!findNextJoint)
					{
						branchEnd = true;
					}
				}
			}
		}else
		{
			branchEnd = true;
		}
	} while (!branchEnd);
}


/* separate branches from a whole skeleton set
[in] vector<voxelKey>& skeletonSet
[out] vector<vector<voxelKey>>& branches
*/
void skeletonAnalysis::separateBranch(vector<voxelKey> skeletonSet, vector<branch>& branches)
{
	cout << "\nseparate branches from the skeleton ...";
	pcl::StopWatch timer;
	vector<voxelKey> jointpoints;
	vector<voxelKey> trueJoint;
	findJointpoints(skeletonSet, jointpoints, trueJoint);
	if (jointpoints.size() > 0) 
	{
		for (voxelKey start : trueJoint)
		{
			int branchNumber = getTargetSurroundings(start, skeletonSet).allNeighborSet.size();
			for (int i = 0; i < branchNumber; i++)
			{
				branch oneBranch;
				getOneBranch(start, jointpoints, trueJoint, skeletonSet, oneBranch.branckKey);
				estimateEllipticalRatio(oneBranch.branckKey, oneBranch.ellipticalRatio);
				estimateRadius(oneBranch.branckKey, 1, oneBranch.voxelRadius);
				branches.push_back(oneBranch);
			}
		}
	}
	else // skeleton only has one branch
	{
		branch oneBranch;
		oneBranch.branckKey = skeletonSet;
		estimateEllipticalRatio(oneBranch.branckKey, oneBranch.ellipticalRatio);
		estimateRadius(oneBranch.branckKey, 1, oneBranch.voxelRadius);
		branches.push_back(oneBranch);
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << "joint points: " << trueJoint.size() << endl;
	cout << "branches: " << branches.size() << endl;
}


