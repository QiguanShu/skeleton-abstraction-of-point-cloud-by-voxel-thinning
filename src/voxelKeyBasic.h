#pragma once
#include "voxelViewer.h"
using namespace std;

// basic set calculations
bool belongsToSet(voxelKey& key, vector<voxelKey>& set);
bool isSubset(vector<voxelKey>& set1, vector<voxelKey>& set2);
bool isEqual(vector<voxelKey>& set1, vector<voxelKey>& set2);
void setIntersection(vector<voxelKey>& set1, vector<voxelKey>& set2, vector<voxelKey>& interSet);
void setUnion(vector<voxelKey>& set1, vector<voxelKey>& set2, vector<voxelKey>& unionSet);
void relativeComplement(vector<voxelKey>& set1, vector<voxelKey>& set2, vector<voxelKey>& compleSet);
void setDifference(vector<voxelKey>& set1, vector<voxelKey>& set2);
void setDifference(vector<voxelKey>& set, voxelKey key);
void removeDuplicate(vector<voxelKey>& set);

// spatial relationships between voxels
void rangeSet(voxelKey& minKey, voxelKey& maxKey, vector<voxelKey>& rangeSet);
void universalSet(voxelKey& boundaryKey, vector<voxelKey>& uniSet);
void get26Surroundings(voxelKey& inspectKey, vector<voxelKey>& surroundingSet);
class getTargetSurroundings
{
public:
	getTargetSurroundings(voxelKey& inspectKey, vector<voxelKey>& targetSet);
	vector<voxelKey> F_NeighborSet;
	vector<voxelKey> E_NeighborSet;
	vector<voxelKey> V_NeighborSet;
	vector<voxelKey> allNeighborSet;
};

// continuous voxel block
void getConnected(voxelKey& inspectKey, vector<voxelKey>& targetSet, const int type, vector<voxelKey>& connectedSet); // type = 6: face connectivity; type = 18: face + edge connectivity; type = 26: face + edge + vertex connectivity; 
void removeSeparate(vector<voxelKey>& set, const int type);