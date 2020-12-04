#include "voxelKeyBasic.h"

class voxel3DThinning
{
public:
	// main function
	voxel3DThinning(voxelObject& inputVoxelSet);
	voxelObject voxelSet;
	vector<voxelKey> P_Set;
	vector<voxelKey> R_Set;
	vector<voxelKey> oneSubDelete; // deleted voxels in a subiteration of one template in one direction
	vector<voxelKey> allSubDelete; // all deleted voxels in subiterations within one main iteration
	vector<voxelKey> edgeKeySet; // voxels at the boundary edge
	int iteration = 0;
	struct cubeTemplate
	{
		vector<int> objectID;
		vector<int> voidID;
	};
	
private:
	// subfunctions - identify templates
	cubeTemplate tempA;
	cubeTemplate tempB;
	cubeTemplate tempC;
	cubeTemplate tempD;
	vector<cubeTemplate> allTempA;
	vector<cubeTemplate> allTempB;
	vector<cubeTemplate> allTempC;
	vector<cubeTemplate> allTempD;
	void cubeIDToKey(voxelKey& inspectKey, vector<int>& cubeIDSet, vector<voxelKey>& keySet);
	void keyToCubeID(voxelKey& inspectKey, vector<voxelKey>& keySet, vector<int>& cubeIDSet);
	void rotateCubeID(vector<int>& idSet,string axis, int angle, vector<int>& rotatedIdSet);
	void rotateCubeID(vector<int>& idSet, string axis, int angle);
	bool IDbelongsToSet(int& id, vector<int>& idSet);
	void rotateTemplate(cubeTemplate& inputTemplate, string axis, int angle, cubeTemplate& rotatedTemplate);
	void rotateTemplate(cubeTemplate& inputTemplate, string axis, int angle);
	bool belongsToTemplate(cubeTemplate& inputTemp, vector<cubeTemplate>& allTemplates);
	void prefabTemplates(cubeTemplate inputTemp, vector<cubeTemplate>& allTemplates);
	bool isTemplate(cubeTemplate& inputTemp, cubeTemplate temp);
	bool isTemplateAllDirections(vector<voxelKey>& objectNeighborSet, vector<voxelKey>& voidNeighborSet, voxelKey& inspectKey, vector<cubeTemplate>& allTemp);

	// subfunctions - delete criteria
	bool deleteCriteria1(vector<voxelKey>& R_Set, voxelKey& inspectKey);
	bool deleteCriteria2(vector<voxelKey>& F_VoidNeighborSet, vector<voxelKey>& E_VoidNeighborSet, voxelKey& inspectKey);
	bool deleteCriteria3(vector<voxelKey>& P_Set, vector<voxelKey>& R_Set, voxelKey& inspectKey);
	bool deleteCriteria4(vector<voxelKey>& P_Set, vector<voxelKey>& allVoidNeighborSet, voxelKey& inspectKey);
	void deleteSubiteration(vector<cubeTemplate> allTemp);

	// subfunctions - delete voxels at the boundary edge
	void cutOffBoundary(voxelObject& voxelSet, vector<voxelKey>& edgeKeySet);

	// subfunctions - display iteration of exposure
	void viewExposure(int& totalIteration, int faceNumber);
};