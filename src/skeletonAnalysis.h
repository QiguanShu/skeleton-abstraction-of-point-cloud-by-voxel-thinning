#include "voxelKeyBasic.h"

class skeletonAnalysis
{
public:
	skeletonAnalysis(voxelObject& skeletonSet, voxelObject& voxelModelSet);
	voxelObject rebuiltModel;
	vector<voxelKey> commonSet;
	vector<voxelKey> missingSet;
	vector<voxelKey> redundantSet;

private:
	void estimateEllipticalRatio(vector<voxelKey>& skeletonKey, float& ellipticalRatio);
	void estimateRadius(vector<voxelKey>& skeletonKey, const int type, float& voxelRadius);
	void expandSkeletonKey(voxelKey& skeletonKey, const int type);
	void expandSkeletonKey(voxelKey& skeletonKey, const int type, float& ellipticalRatio);
	void rebuildModel(voxelObject& skeletonSet, const int type, vector<voxelKey>& voxelModelSet);
	void rebuildModel(voxelObject& skeletonSet, const int type, vector<branch>branches, vector<voxelKey>& voxelModelSet);
	void modelCompare(vector<voxelKey>& voxelModelSet, vector<voxelKey>& rebuiltModel);
	void findTrueJoint(vector<voxelKey>& jointGroup, voxelKey& trueJoint);
	void findJointpoints(vector<voxelKey>& skeletonSet, vector<voxelKey>& jointpoints, vector<voxelKey>& trueJoint);
	void getOneBranch(voxelKey& start, vector<voxelKey>& jointpoints, vector<voxelKey>& trueJoint, vector<voxelKey>& skeletonSet, vector<voxelKey>& branchSet);
	void separateBranch(vector<voxelKey> skeletonSet, vector<branch>& branches);
};