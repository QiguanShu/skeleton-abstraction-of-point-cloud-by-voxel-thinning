#include "voxelKeyBasic.h"


/* tell if a voxel key belongs to a key set
[in] voxelKey& key
[in] vector<voxelKey>& set
[return] boolean values
*/
bool belongsToSet(voxelKey& key, vector<voxelKey>& set)
{
	for (voxelKey setKey : set)
	{
		if (key.x == setKey.x && key.y == setKey.y && key.z == setKey.z)
		{
			return true;
		}
	}
	return false;
}


/* tell if set 1 is a subset of set 2
[in] vector<voxelKey>& set1
[in] vector<voxelKey>& set2
[return] boolean values
*/
bool isSubset(vector<voxelKey>& set1, vector<voxelKey>& set2)
{
	for (voxelKey key : set1)
	{
		if (!belongsToSet(key, set2))
		{
			return false;
		}
	}
	return true;
}


/* tell if set 1 is equal to set 2
[in] vector<voxelKey>& set1
[in] vector<voxelKey>& set2
[return] boolean values
*/
bool isEqual(vector<voxelKey>& set1, vector<voxelKey>& set2)
{
	if (set1.size() != set2.size())
	{
		return false;
	}
	if (isSubset(set1, set2))
	{
		return true;
	}
	return false;
}


/* intersection of set 1 and set 2
[in] vector<voxelKey>& set1
[in] vector<voxelKey>& set2
[out] vector<voxelKey>& interSet
*/
void setIntersection(vector<voxelKey>& set1, vector<voxelKey>& set2, vector<voxelKey>& interSet)
{
	interSet.clear();
	for (voxelKey key : set1)
	{
		if (belongsToSet(key, set2))
		{
			interSet.push_back(key);
		}
	}
}


/* union of set 1 and set 2
[in] vector<voxelKey>& set1
[in] vector<voxelKey>& set2
[out] vector<voxelKey>& unionSet
*/
void setUnion(vector<voxelKey>& set1, vector<voxelKey>& set2, vector<voxelKey>& unionSet)
{
	unionSet.clear();
	for (voxelKey key : set1)
	{
		if (!belongsToSet(key, set2))
		{
			unionSet.push_back(key);
		}
	}
	for (voxelKey key : set2)
	{
		unionSet.push_back(key);
	}
}


/* get keys that belong to set 1 while not belong to set 2
[in] vector<voxelKey>& set1
[in] vector<voxelKey>& set2
[out] vector<voxelKey>& compleSet
*/
void relativeComplement(vector<voxelKey>& set1, vector<voxelKey>& set2, vector<voxelKey>& compleSet)
{
	compleSet.clear();
	for (voxelKey key : set1)
	{
		if (!belongsToSet(key, set2))
		{
			compleSet.push_back(key);
		}
	}
}


/* subtract set 2 from set 1
[in] vector<voxelKey>& set1
[in] vector<voxelKey>& set2
[out] vector<voxelKey>& set1
*/
void setDifference(vector<voxelKey>& set1, vector<voxelKey>& set2)
{
	vector<voxelKey> newSet1;
	relativeComplement(set1, set2, newSet1);
	set1.clear();
	for (voxelKey key : newSet1)
	{
		set1.push_back(key);
	}
}


/* subtract a key from set
[in] vector<voxelKey>& set1
[in] vector<voxelKey>& set2
[out] vector<voxelKey>& set1
*/
void setDifference(vector<voxelKey>& set,voxelKey key)
{
	vector<voxelKey> newSet;
	for(voxelKey newKey : set)
	{
		if(newKey.x != key.x || newKey.y != key.y || newKey.z != key.z)
		{
			newSet.push_back(newKey);
		}
	}
	set.clear();
	for (voxelKey newKey : newSet)
	{
		set.push_back(newKey);
	}
}


/* remove duplicates in a set
[in] vector<voxelKey>& set
[out] vector<voxelKey>& set
*/
void removeDuplicate(vector<voxelKey>& set)
{
	vector<voxelKey> newSet;
	for (int pos1 = 0; pos1 < set.size() - 1; pos1++)
	{
		for (int pos2 = pos1 + 1; pos2 < set.size(); pos2++)
		{
			if (set[pos1].x == set[pos2].x && set[pos1].y == set[pos2].y && set[pos1].z == set[pos2].z)
			{
				break;
			}
			if (pos2 == set.size() - 1)
			{
				newSet.push_back(set[pos1]);
			}
		}
	}
	newSet.push_back(set[set.size() - 1]);
	set.clear();
	for (voxelKey key : newSet)
	{
		set.push_back(key);
	}
}


/* get the key set defined by minimum key and maximum key
[in] voxelKey& minKey
[in] voxelKey& maxKey
[out] vector<voxelKey>& rangeSet
*/
void rangeSet(voxelKey& minKey, voxelKey& maxKey, vector<voxelKey>& rangeSet)
{
	rangeSet.clear();
	for (int x = minKey.x; x <= maxKey.x; x++)
	{
		for (int y = minKey.y; y <= maxKey.y; y++)
		{
			for (int z = minKey.z; z <= maxKey.z; z++)
			{
				voxelKey key;
				key.x = x;
				key.y = y;
				key.z = z;
				rangeSet.push_back(key);
			}
		}
	}
}


/* get the universal key set defined by a boundary key
[in] voxelKey& boundaryKey
[out] vector<voxelKey>& uniSet
*/
void universalSet(voxelKey& boundaryKey, vector<voxelKey>& uniSet)
{
	uniSet.clear();
	voxelKey zeroKey;
	zeroKey.x = 0;
	zeroKey.y = 0;
	zeroKey.z = 0;
	rangeSet(zeroKey, boundaryKey, uniSet);
}


/* get all keys surrounding the inspecting key
[in] voxelKey& inspectKey
[out] vector<voxelKey>& surroundingSet
*/
void get26Surroundings(voxelKey& inspectKey, vector<voxelKey>& surroundingSet)
{
	surroundingSet.clear();
	for (int x = inspectKey.x - 1; x <= inspectKey.x + 1; x++)
	{
		for (int y = inspectKey.y - 1; y <= inspectKey.y + 1; y++)
		{
			for (int z = inspectKey.z - 1; z <= inspectKey.z + 1; z++)
			{
				voxelKey key;
				key.x = x;
				key.y = y;
				key.z = z;
				if (key.x != inspectKey.x || key.y != inspectKey.y || key.z != inspectKey.z)
				{
					surroundingSet.push_back(key);
				}
			}
		}
	}
}


/* get all keys of a target group surrounding the inspected key
[in] voxelKey& inspectKey
[in] vector<voxelKey>& targetSet
[out] vector<voxelKey>& F_NeighborSet
[out] vector<voxelKey>& E_NeighborSet
[out] vector<voxelKey>& V_NeighborSet
[out] vector<voxelKey>& allNeighborSet
*/
getTargetSurroundings::getTargetSurroundings(voxelKey& inspectKey, vector<voxelKey>& targetSet)
{
	F_NeighborSet.clear();
	E_NeighborSet.clear();
	V_NeighborSet.clear();
	allNeighborSet.clear();
	for (voxelKey key : targetSet)
	{
		if (labs(key.x - inspectKey.x) <= 1 && labs(key.y - inspectKey.y) <= 1 && labs(key.z - inspectKey.z) <= 1)
		{
			int sameXYZ = 0;
			if (key.x == inspectKey.x)
			{
				sameXYZ++;
			}
			if (key.y == inspectKey.y)
			{
				sameXYZ++;
			}
			if (key.z == inspectKey.z)
			{
				sameXYZ++;
			}
			switch(sameXYZ)
			{
			case 0:
				V_NeighborSet.push_back(key);
				break;
			case 1: 
				E_NeighborSet.push_back(key);
				break;
			case 2:
				F_NeighborSet.push_back(key);
				break;
			}		
			if (sameXYZ != 3)
			{
				allNeighborSet.push_back(key);
			}
		}
	}
}


/* get keys that are directly or indirectly connected to the inspecting key
[in] voxelKey& inspectKey
[in] voxelKey& targetSet
[in] int type (type = 6: face connectivity; type = 18: face + edge connectivity; type = 26: face + edge + vertex connectivity)
[out] vector<voxelKey>& connectedSet
*/
void getConnected(voxelKey& inspectKey, vector<voxelKey>& targetSet, const int type, vector<voxelKey>& connectedSet)
{
	if(targetSet.size() > 26)
	{
		cout << "calculate connectivity ...       ";
	}
	connectedSet.clear();
	connectedSet.push_back(inspectKey);
	vector<voxelKey> checkedKey;
	int id = 0;
	do
	{
		getTargetSurroundings neighborSet(connectedSet[id], targetSet);
		checkedKey.push_back(connectedSet[id]);
		vector<voxelKey> connectingRange;
		switch(type)
		{
		case 6:
			connectingRange = neighborSet.F_NeighborSet;
			break;
		case 18: 
			setUnion(neighborSet.F_NeighborSet, neighborSet.E_NeighborSet, connectingRange);
			break;
		case 26: 
			connectingRange = neighborSet.allNeighborSet;
			break;
		}
		for (voxelKey key : connectingRange)
		{
			if (!belongsToSet(key, connectedSet))
			{
				connectedSet.push_back(key);
			}
			if (targetSet.size() > 26)
			{
				printf("\b\b\b\b\b\b\b%05.1f%% ", (float)connectedSet.size() / (float)targetSet.size() * (float)100);
			}
		}
		id++;
	} while (checkedKey.size() < connectedSet.size());
}


/* delete separate voxels from the set and remian the main connecting body (connected voxels are more than the half of the set)
[in] vector<voxelKey>& set
[int] type (type = 6: face connectivity; type = 18: face + edge connectivity; type = 26: face + edge + vertex connectivity)
[out] vector<voxelKey>& set
*/
void removeSeparate(vector<voxelKey>& set, const int type)
{
	voxelKey inspectKey;
	vector<voxelKey> connectedSet;
	int inputSize = set.size();
	do
	{
		inspectKey = set[0];
		getConnected(inspectKey, set, type, connectedSet);
		if (connectedSet.size() < set.size() / 2)
		{
			// cut off connectedSet from input set
			setDifference(set, connectedSet);
		}
		else
		{
			set.clear();
			for (voxelKey key : connectedSet)
			{
				set.push_back(key);
			}
			return;
		}
	} while (set.size() < inputSize / 2);
	cout << "error: input set does not have a main connective body" << endl;
}


