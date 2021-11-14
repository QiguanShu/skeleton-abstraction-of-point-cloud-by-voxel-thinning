#include "voxel3DThinning.h"
#include "fileManager.h"


/* perform 3D thinning of a object voxel set
[in] vector<voxelKey>& objectSet
[in] pcl::PointXYZ originPoint
[in] double resolution
[out] vector<voxelKey>& objectSet
*/
voxel3DThinning::voxel3DThinning(voxelObject& inputVoxelSet):
	voxelSet(inputVoxelSet)
{
	// check if .skl is existed
	if(!readSkl(voxelSet.fileName, voxelSet.resolution,voxelSet))
	{
		// initialize template A, B, C, D
		tempA.objectID.clear();
		tempA.voidID.clear();
		tempA.objectID = { 16 };
		tempA.voidID = { 0, 1, 2, 9, 10, 11, 18, 19, 20 };
		tempB.objectID.clear();
		tempB.voidID.clear();
		tempB.objectID = { 4, 16 };
		tempB.voidID = { 9, 10, 11, 18, 19, 20, 21, 22, 23 };
		tempC.objectID.clear();
		tempC.voidID.clear();
		tempC.objectID = { 4, 12, 16 };
		tempC.voidID = { 10, 11, 14, 19, 20, 22, 23 };
		tempD.objectID.clear();
		tempD.voidID.clear();
		tempD.objectID = { 7 };
		tempD.voidID = { 1, 4, 10, 16, 19, 22, 25 };
		cout << "template A - ";
		prefabTemplates(tempA, allTempA);
		cout << "template B - ";
		prefabTemplates(tempB, allTempB);
		cout << "template C - ";
		prefabTemplates(tempC, allTempC);
		cout << "template D - ";
		prefabTemplates(tempD, allTempD);
				
		// 3D thinning iteration beginns
		do
		{
			allSubDelete.clear();
			iteration++;
			cout << "\n\n3D thinning - the " << iteration << " th iteration" << endl;
			
			// step 1: object voxels that belongs to either template A, B, C, or D go to P_set, the others go to R_set
			cout << "\n3D voxel thinning step 1: identify voxels by templates ...";
			pcl::StopWatch timer;
			P_Set.clear();
			R_Set.clear();
			for (voxelKey key : voxelSet.objectSet)
			{
				getTargetSurroundings objectNeighborSet(key, voxelSet.objectSet);
				getTargetSurroundings voidNeighborSet(key, voxelSet.voidSet);
				key.exposureIteration[0] = voidNeighborSet.F_NeighborSet.size(); // record the face exposure before thinning
				if (voidNeighborSet.allNeighborSet.size() > 0 && (
					isTemplateAllDirections(objectNeighborSet.allNeighborSet, voidNeighborSet.allNeighborSet, key, allTempA) ||
					isTemplateAllDirections(objectNeighborSet.allNeighborSet, voidNeighborSet.allNeighborSet, key, allTempB) ||
					isTemplateAllDirections(objectNeighborSet.allNeighborSet, voidNeighborSet.allNeighborSet, key, allTempC) ||
					isTemplateAllDirections(objectNeighborSet.allNeighborSet, voidNeighborSet.allNeighborSet, key, allTempD)))
				{
					P_Set.push_back(key);
				}
				else
				{
					R_Set.push_back(key);
				}
			}
			cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
			cout << "total P set voxels: " << P_Set.size() << endl;
			cout << "total R set voxels: " << R_Set.size() << endl;
			// view the P set and R set voxels
			/*
			voxelViewer viewPRSet(voxelSet);
			color yellow;
			yellow.r = 1.0;
			yellow.g = 0.8;
			yellow.b = 0.2;
			color blue;
			blue.r = 0.2;
			blue.g = 0.8;
			blue.b = 1.0;
			viewPRSet.addVoxelKey(P_Set, 1, yellow, false, "PSet");
			viewPRSet.addVoxelKey(R_Set, 1, blue, false, "RSet");
			viewPRSet.run();
			*/
			
			// step 2: check if P_set voxels are p-simple (satisfy all 4 deleting criteria)
			cout << "\n3D voxel thinning step 2: check if voxels from P set should be eroded ..." << endl;
			timer.reset();
			// template A subiterations
			cout << "deleting template A";
			deleteSubiteration(allTempA);
			// template B subiterations
			cout << "deleting template B";
			deleteSubiteration(allTempB);
			// template C subiterations
			cout << "deleting template C";
			deleteSubiteration(allTempC);
			// template D subiterations
			cout << "deleting template D";
			deleteSubiteration(allTempD);
			cout << "total deleting voxels in all subiterations: " << allSubDelete.size() << endl;
			cout << "remaining object voxels after all subiterations: " << voxelSet.objectSet.size() << endl;
			// view the result
			/*
			// rendering the result in every 2 iterations
			if(iteration % 2 == 1)
			{
				voxelViewer viewDelete(voxelSet);
				color yellow;
				yellow.r = 1.0;
				yellow.g = 0.8;
				yellow.b = 0.2;
				color blue;
				blue.r = 0.2;
				blue.g = 0.8;
				blue.b = 1.0;
				// alternative to view the section
				///*
				vector<voxelKey> sectionSet, deletingSet;
				for(voxelKey key : voxelSet.objectSet)
				{
					if(key.z == voxelSet.boundaryKey.z / 2)
					{
						sectionSet.push_back(key);
					}
				}
				for (voxelKey key : allSubDelete)
				{
					if (key.z == voxelSet.boundaryKey.z / 2)
					{
						deletingSet.push_back(key);
					}
				}
				viewDelete.addVoxelKey(sectionSet, 1, blue, false, "objectSet");
				viewDelete.addVoxelKey(deletingSet, 1, yellow, true, "deleteSet");
				//
				viewDelete.addVoxelKey(voxelSet.objectSet, 1, blue, false, "objectSet");
				viewDelete.addVoxelKey(allSubDelete, 1, yellow, true, "deleteSet");
				viewDelete.run();
			}
			*/
			
			// step 3: record the exposure iteration
			cout << "\n3D voxel thinning step 3: note down the exposure iteration ...";
			timer.reset();
			for (voxelKey& key : voxelSet.objectSet)
			{
				int F_Exposure = 6 - getTargetSurroundings(key, voxelSet.objectSet).F_NeighborSet.size();
				for (int i = 1; i < 5; i++)
				{
					if (F_Exposure < i)
					{
						key.exposureIteration[i]++;
					}
				}
			}
			cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
		} while (allSubDelete.size() > 0);

		// cut off voxels at the boundary edge
		cutOffBoundary(voxelSet, edgeKeySet);

		//save the skeleton
		for (voxelKey& key : voxelSet.objectSet)
		{
			for (int i = 1; i < 5; i++)
			{
				key.exposureIteration[i]++;
			}
		}
		voxelSet.totalIteration = iteration;
		writeSkl(voxelSet);
	}

	//view skeleton result
	cout << "\nfinal skeleton result ...";
	voxelViewer viewSkeleton(voxelSet);
	color blue;
	blue.r = 0.2;
	blue.g = 0.8;
	blue.b = 1.0;
	color yellow;
	yellow.r = 1.0;
	yellow.g = 0.8;
	yellow.b = 0.2;
	viewSkeleton.addVoxelKey(voxelSet.objectSet, 1, blue, false, "objectSet");
	viewSkeleton.addVoxelKey(edgeKeySet, 1, yellow, true, "voidSet");
	viewSkeleton.run();

	// view the exposure iteration
	///*
	viewExposure(voxelSet.totalIteration, 1);
	viewExposure(voxelSet.totalIteration, 2);
	viewExposure(voxelSet.totalIteration, 3);
	viewExposure(voxelSet.totalIteration, 4);
	//*/
}


/* interpret cube ID to voxel keys surrounding the inspect key
[in] voxelKey& inspectKey
[in] vector<int>& cubeIDSet
[out] vector<voxelKey>& keySet
*/
void voxel3DThinning::cubeIDToKey(voxelKey& inspectKey, vector<int>& cubeIDSet, vector<voxelKey>& keySet)
{
	for (int i : cubeIDSet)
	{
		voxelKey key;
		// x axis
		if (i <= 8)
		{
			key.x = inspectKey.x - 1;
		}
		else if (i >= 9 && i <= 17)
		{
			key.x = inspectKey.x;
		}
		else
		{
			key.x = inspectKey.x + 1;
		}
		// y axis
		if (i % 3 == 0)
		{
			key.y = inspectKey.y - 1;
		}
		else if (i % 3 == 1)
		{
			key.y = inspectKey.y;
		}
		else
		{
			key.y = inspectKey.y - 1;
		}
		// z axis
		if (i % 9 == 0 || i % 9 == 1 || i % 9 == 2)
		{
			key.z = inspectKey.z + 1;
		}
		else if (i % 9 == 3 || i % 9 == 4 || i % 9 == 5)
		{
			key.z = inspectKey.z;
		}
		else
		{
			key.z = inspectKey.z - 1;
		}
		keySet.push_back(key);
	}
}


/* interpret voxel keys to cube ID surrounding the inspect key
[in] voxelKey& inspectKey
[in] vector<voxelKey>& keySet
[out] vector<int>& cubeIDSet
*/
void voxel3DThinning::keyToCubeID(voxelKey& inspectKey, vector<voxelKey>& keySet, vector<int>& cubeIDSet)
{
	for (voxelKey key : keySet)
	{
		int a, b, c;
		// x axis
		if (key.x == inspectKey.x - 1)
		{
			a = 0;
		}
		else if (key.x == inspectKey.x)
		{
			a = 1;
		}
		else
		{
			a = 2;
		}
		// y axis
		if (key.y == inspectKey.y + 1)
		{
			b = 0;
		}
		else if (key.y == inspectKey.y)
		{
			b = 1;
		}
		else
		{
			b = 2;
		}
		// z axis
		if (key.z == inspectKey.z + 1)
		{
			c = 0;
		}
		else if (key.z == inspectKey.z)
		{
			c = 1;
		}
		else
		{
			c = 2;
		}
		cubeIDSet.push_back(a * 9 + b + c * 3);
	}
}


/* rotate cube id set in one axis for given times of 90 degree
[in] vector<int>& idSet
[in] string axis
[in] int angle (integer multiple 90 degrees)
[out] vector<int> & rotatedIdSet
*/
void voxel3DThinning::rotateCubeID(vector<int> & idSet, string axis, int angle, vector<int> & rotatedIdSet)
{
	rotatedIdSet.clear();
	for (int i : idSet)
	{
		int a, b, c;
		a = i / 9;
		c = (i - a * 9) / 3;
		b = i - a * 9 - c * 3;
		int temp;
		if (axis == "x")
		{
			for (int n = 0; n < angle; n++)
			{
				temp = b;
				b = c;
				c = 2 - temp;
			}
		}
		if (axis == "y")
		{
			for (int n = 0; n < angle; n++)
			{
				temp = a;
				a = c;
				c = 2 - temp;
			}
		}
		if (axis == "z")
		{
			for (int n = 0; n < angle; n++)
			{
				temp = a;
				a = b;
				b = 2 - temp;
			}
		}
		rotatedIdSet.push_back(a * 9 + b + c * 3);
	}
}


/* rotate cube id set in one axis for given times of 90 degree
[in] vector<int>& idSet
[in] string axis
[in] int angle (integer multiple 90 degrees)
[out] vector<int>& idSet
*/
void voxel3DThinning::rotateCubeID(vector<int> & idSet, string axis, int angle)
{
	vector<int> rotatedIdSet;
	for (int i : idSet)
	{
		int a, b, c;
		a = i / 9;
		c = (i - a * 9) / 3;
		b = i - a * 9 - c * 3;
		int temp;
		if (axis == "x")
		{
			for (int n = 0; n < angle; n++)
			{
				temp = b;
				b = c;
				c = 2 - temp;
			}
		}
		if (axis == "y")
		{
			for (int n = 0; n < angle; n++)
			{
				temp = a;
				a = c;
				c = 2 - temp;
			}
		}
		if (axis == "z")
		{
			for (int n = 0; n < angle; n++)
			{
				temp = a;
				a = b;
				b = 2 - temp;
			}
		}
		rotatedIdSet.push_back(a * 9 + b + c * 3);

		idSet.clear();
		for (int i : rotatedIdSet)
		{
			idSet.push_back(i);
		}
	}
}


/* rotate template by giving rotation axis and angle
[in] cubeTemplate& inputTemplate
[in] string axis
[in] int angle  (integer multiple 90 degrees)
[out] cubeTemplate& rotatedTemplate
*/
void voxel3DThinning::rotateTemplate(cubeTemplate& inputTemplate, string axis, int angle, cubeTemplate& rotatedTemplate)
{
	rotatedTemplate.objectID.clear();
	rotatedTemplate.voidID.clear();
	rotateCubeID(inputTemplate.objectID, axis, angle, rotatedTemplate.objectID);
	rotateCubeID(inputTemplate.voidID, axis, angle, rotatedTemplate.voidID);
}


/* rotate template by giving rotation axis and angle
[in] cubeTemplate& inputTemplate
[in] string axis
[in] int angle (integer multiple 90 degrees)
[out] cubeTemplate& inputTemplate
*/
void voxel3DThinning::rotateTemplate(cubeTemplate & inputTemplate, string axis, int angle)
{
	rotateCubeID(inputTemplate.objectID, axis, angle);
	rotateCubeID(inputTemplate.voidID, axis, angle);
}


/* if an id belongs to an id set
[in] int& id
[in] vector<int>& idSet
[return] boolean value
*/
bool voxel3DThinning::IDbelongsToSet(int& id, vector<int>& idSet)
{
	for (int i : idSet)
	{
		if (i == id)
		{
			return true;
		}
	}
	return false;
}


/* if an template belongs to an template set
[in] cubeTemplate inputTemp
[in] vector<cubeTemplate> allTemplates
[return] boolean value
*/
bool voxel3DThinning::belongsToTemplate(cubeTemplate& inputTemp, vector<cubeTemplate>& allTemplates)
{
	bool existed = false;
	for (cubeTemplate temp : allTemplates)
	{
		bool sameTemp = true;
		for (int objectID : temp.objectID)
		{
			if (!IDbelongsToSet(objectID, inputTemp.objectID))
			{
				sameTemp = false;
				break;
			}
		}
		if (sameTemp)
		{
			existed = true;
			break;
		}
	}
	if (existed)
	{
		return true;
	}
	return false;
}


/* storage unique template of each direction to the vector allTemplates
[in] cubeTemplate inputTemp
[out] vector<cubeTemplate> allTemplates
*/
void voxel3DThinning::prefabTemplates(cubeTemplate inputTemp, vector<cubeTemplate>& allTemplates)
{
	cout << "prefabricate and store template in all possible directions ... ";
	// rotate along x axis
	int xRotation = 0;
	do
	{
		// rotate along z axis
		int zRotation = 0;
		do
		{
			cubeTemplate rotatedTemplate;
			rotateTemplate(inputTemp, "z", zRotation, rotatedTemplate);
			if (allTemplates.size() == 0 || !belongsToTemplate(rotatedTemplate, allTemplates))
			{
				allTemplates.push_back(rotatedTemplate);
			}
			zRotation++;
		} while (zRotation < 4);
		rotateTemplate(inputTemp, "x", 1);
		xRotation++;
	} while (xRotation < 3);
	cout << "final prefabricate templates: " << allTemplates.size() << endl;
}


/* tell if a pair of object ID and void Id is the given template in this given direction
[in] vector<int>& objectIdSet
[in] vector<int>& voidIdSet
[in] cubeTemplate temp
[return] boolean values
*/
bool voxel3DThinning::isTemplate(cubeTemplate& inputTemp, cubeTemplate temp)
{
	bool isTemp = true;
	for (int obj : temp.objectID)
	{
		if (!IDbelongsToSet(obj, inputTemp.objectID))
		{
			isTemp = false;
			break;
		}
	}
	for (int voi : temp.voidID)
	{
		if (!IDbelongsToSet(voi, inputTemp.voidID))
		{
			isTemp = false;
			break;
		}
	}
	if (isTemp)
	{
		return true;
	}
	return false;
}


/* tell if the inspect key and its surroundings belongs to the given template in all directions
[in] vector<voxelKey>& objectNeighborSet
[in] vector<voxelKey>& voidNeighborSet
[in] voxelization::voxelKey& inspectKey
[in] cubeTemplate temp
[return] boolean values
*/
bool voxel3DThinning::isTemplateAllDirections(vector<voxelKey>& objectNeighborSet, vector<voxelKey>& voidNeighborSet, voxelKey& inspectKey, vector<cubeTemplate>& allTemp)
{
	cubeTemplate inputTemp;
	keyToCubeID(inspectKey, objectNeighborSet, inputTemp.objectID);
	keyToCubeID(inspectKey, voidNeighborSet, inputTemp.voidID);
	for(cubeTemplate temp : allTemp)
	{
		if (isTemplate(inputTemp, temp))
		{
			return true;
		}
	}
	return false;
}


/* tell if the inspect key and its surroundings satisfy delete criteria 1
[in] vector<voxelKey>& R_Set
[in] voxelKey& inspectKey
[return] boolean values
*/
bool voxel3DThinning::deleteCriteria1(vector<voxelKey>& R_Set, voxelKey& inspectKey)
{
	vector<voxelKey> connectedSet;
	getTargetSurroundings R_Surroundings(inspectKey, R_Set);
	if(R_Surroundings.allNeighborSet.size() == 0)
	{
		return false;
	}
	getConnected(R_Surroundings.allNeighborSet[0], R_Surroundings.allNeighborSet, 26, connectedSet);
	if(connectedSet.size() == R_Surroundings.allNeighborSet.size())
	{
		return true;
	}
	return false;
}


/* tell if the inspect key and its surroundings satisfy delete criteria 2
[in] vector<voxelKey>& F_VoidNeighborSet
[in] vector<voxelKey>& E_VoidNeighborSet
[in] voxelKey& inspectKey
[return] boolean values
*/
bool voxel3DThinning::deleteCriteria2(vector<voxelKey>& F_VoidNeighborSet, vector<voxelKey>& E_VoidNeighborSet, voxelKey& inspectKey)
{
	vector<voxelKey> void18NeighborSet;
	for(voxelKey key : F_VoidNeighborSet)
	{
		void18NeighborSet.push_back(key);
	}
	for (voxelKey key : E_VoidNeighborSet)
	{
		void18NeighborSet.push_back(key);
	}
	if(void18NeighborSet.size() > 0)
	{
		if(F_VoidNeighborSet.size() > 0)
		{
			if (F_VoidNeighborSet.size() == 1)
			{
				return true;
			}
			else
			{
				vector<voxelKey> connectedSet;
				getConnected(F_VoidNeighborSet[0], void18NeighborSet,6, connectedSet);
				setDifference(void18NeighborSet, connectedSet);
				for (voxelKey key : void18NeighborSet)
				{
					if (belongsToSet(key, F_VoidNeighborSet))
					{
						return false;
					}
				}
				return true;
			}
		}
	}
	return false;
}


/* tell if the inspect key with its surroundings satisfy delete criteria 3
[in] vector<voxelKey>& P_Set
[in] vector<voxelKey>& R_Set
[in] voxelKey& inspectKey
[return] boolean values
*/
bool voxel3DThinning::deleteCriteria3(vector<voxelKey>& P_Set, vector<voxelKey>& R_Set, voxelKey& inspectKey)
{
	getTargetSurroundings P_Surroundings(inspectKey, P_Set);
	getTargetSurroundings R_Surroundings(inspectKey, R_Set);
	for(voxelKey PKey : P_Surroundings.allNeighborSet)
	{
		bool adjacentR = false;
		for (voxelKey RKey : R_Surroundings.allNeighborSet)
		{
			vector<voxelKey> surroundingSet;
			get26Surroundings(PKey, surroundingSet);			
			if (belongsToSet(RKey,surroundingSet))
			{
				adjacentR = true;
				break;
			}
		}
		if(!adjacentR)
		{
			return false;
		}
	}
	return true;
}


/* tell if the inspect key with its surroundings satisfy delete criteria 4
[in] vector<voxelKey>& P_Set
[in] vector<voxelKey>& R_Set
[in] voxelKey& inspectKey
[return] boolean values
*/
bool voxel3DThinning::deleteCriteria4(vector<voxelKey>& P_Set, vector<voxelKey>& allVoidNeighborSet, voxelKey& inspectKey)
{
	getTargetSurroundings P_Surroundings(inspectKey, P_Set);
	if(P_Surroundings.F_NeighborSet.size() == 0)
	{
		return true;
	}
	if (allVoidNeighborSet.size() < 2)
	{
		return false;
	}
	for(voxelKey PKey : P_Surroundings.F_NeighborSet)
	{
		bool unitSquare = false;
		// if the P key is located along x axis with the inspecting key
		if(PKey.x != inspectKey.x)
		{
			for(voxelKey voidKey1 : allVoidNeighborSet)
			{
				for (voxelKey voidKey2 : allVoidNeighborSet)
				{
					if (abs(voidKey1.x - voidKey2.x) == 1 && voidKey1.x == PKey.x && voidKey2.x == inspectKey.x)
					{
						unitSquare = true;
						break;
					}
				}
			}
		}
		// if the P key is located along y axis with the inspecting key
		if (PKey.y != inspectKey.y)
		{
			for (voxelKey voidKey1 : allVoidNeighborSet)
			{
				for (voxelKey voidKey2 : allVoidNeighborSet)
				{
					if (abs(voidKey1.y - voidKey2.y) == 1 && voidKey1.y == PKey.y && voidKey2.y == inspectKey.y)
					{
						unitSquare = true;
						break;
					}
				}
			}
		}
		// if the P key is located along z axis with the inspecting key
		if (PKey.z != inspectKey.z)
		{
			for (voxelKey voidKey1 : allVoidNeighborSet)
			{
				for (voxelKey voidKey2 : allVoidNeighborSet)
				{
					if (abs(voidKey1.z - voidKey2.z) == 1 && voidKey1.z == PKey.z && voidKey2.z == inspectKey.z)
					{
						unitSquare = true;
						break;
					}
				}
			}
		}
		if (!unitSquare)
		{
			return false;
		}
	}
	return true;
}


/* subiterations to delete P_set of one template at all directions
[in] vector<cubeTemplate> allTemp
*/
void voxel3DThinning::deleteSubiteration(vector<cubeTemplate> allTemp)
{
	// template subiterations
	for (cubeTemplate oneTemp : allTemp)
	{
		for (voxelKey key : P_Set)
		{
			if (!belongsToSet(key, allSubDelete))
			{
				getTargetSurroundings objectNeighborSet(key, voxelSet.objectSet);
				getTargetSurroundings voidNeighborSet(key, voxelSet.voidSet);
				cubeTemplate thisCube;
				keyToCubeID(key, objectNeighborSet.allNeighborSet, thisCube.objectID);
				keyToCubeID(key, voidNeighborSet.allNeighborSet, thisCube.voidID);
				if (isTemplate(thisCube, oneTemp))
				{
					if (deleteCriteria1(objectNeighborSet.allNeighborSet, key) &&
						deleteCriteria2(voidNeighborSet.F_NeighborSet, voidNeighborSet.E_NeighborSet, key) &&
						deleteCriteria3(P_Set, objectNeighborSet.allNeighborSet, key) &&
						deleteCriteria4(P_Set, voidNeighborSet.allNeighborSet, key))
					{
						oneSubDelete.push_back(key);
						allSubDelete.push_back(key);
						voxelSet.voidSet.push_back(key);
					}
				}
			}
		}
		// delete voxels from objectSet
		setDifference(voxelSet.objectSet, oneSubDelete);
		cout << " -" << oneSubDelete.size();
		oneSubDelete.clear();
	}
	cout << endl;
}


/* cut off voxels at the boundary edge
[in] voxelObject& voxelSet
[out] voxelObject& voxelSet
[out] vector<voxelKey> edgeKeySet
*/
void voxel3DThinning::cutOffBoundary(voxelObject& voxelSet, vector<voxelKey>& edgeKeySet)
{
	vector<voxelKey> newObjectSet;
	for(voxelKey key : voxelSet.objectSet)
	{
		if(key.x!=0 && key.y!= 0 && key.z!=0 && key.x != voxelSet.boundaryKey.x && key.y != voxelSet.boundaryKey.y && key.z != voxelSet.boundaryKey.z)
		{
			newObjectSet.push_back(key);
		}
		else
		{
			edgeKeySet.push_back(key);
		}
	}
	voxelSet.objectSet.clear();
	voxelSet.objectSet = newObjectSet;
}


/* view a certain number of face exposure iteration
[in] int& totalIteration
[in] faceNumber
*/
void voxel3DThinning::viewExposure(int& totalIteration, int faceNumber)
{
	cout << "\nview the " << faceNumber << " th face exposure iteration ...";
	pcl::StopWatch timer;
	color iterColor;
	iterColor.b = 0.2;
	color gray;
	gray.r = 0.5;
	gray.g = 0.5;
	gray.b = 0.5;
	voxelViewer viewExposure(voxelSet);
	// calculate the display color by counting iteration distribution
	int* voxelCount = new int[totalIteration];
	for (int i = 0; i <= totalIteration; i++)
	{
		voxelCount[i] = 0;
	}
	for (voxelKey key : voxelSet.objectSet)
	{
		for (int i = key.exposureIteration[faceNumber]; i <= totalIteration; i++)
		{
			voxelCount[i]++;
		}
	}
	// display
	for (voxelKey key : voxelSet.objectSet)
	{
		iterColor.r = (double)voxelCount[key.exposureIteration[faceNumber]] / (double)voxelSet.objectSet.size();
		iterColor.g = 1 - iterColor.r;
		getTargetSurroundings objectNeighbor(key, voxelSet.objectSet);
		int totalExposure = 6 - objectNeighbor.F_NeighborSet.size();
		vector<voxelKey> oneKey;
		oneKey.push_back(key);
		if (totalExposure >= faceNumber)
		{
			viewExposure.addVoxelKey(oneKey, 1, iterColor, false, to_string(key.x) + "_" + to_string(key.y) + "_" + to_string(key.z));
		}
		else
		{
			viewExposure.addVoxelKey(oneKey, 1, gray, true, to_string(key.x) + "_" + to_string(key.y) + "_" + to_string(key.z));
		}
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	// print a color legend to the console
	cout << "|------------------------ Color Legend ------------------------|" << endl;
	cout << "| iteration | voxel distribution |    r    |    g    |    b    |" << endl;
	for(int i = 0; i <= totalIteration; i++)
	{
		printf("|    %03i    |        %04i        |   %03.0f   |   %03.0f   |   %03.0f   |\n", i, voxelCount[i],(float)voxelCount[i] / (float)voxelSet.objectSet.size() * 255, 255 - (float)voxelCount[i] / (float)voxelSet.objectSet.size() * 255, 255 * 0.2);
	}
	cout << "|--------------------------------------------------------------|" << endl;
	//delete[] voxelCount; // release the memory
	viewExposure.run();
}

