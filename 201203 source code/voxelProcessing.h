#include "voxelKeyBasic.h"
#include "fileManager.h"
#include "inputBoundaryBox.h"
#include "inputVoxelKey.h"
using namespace std;

class voxelProcessing
{
public:
// main function
voxelProcessing(voxelObject& voxelSet);

private:
// subfunctions
void deleteSeparate(voxelObject& voxelSet, const int type);
void setBoundaryBox(voxelObject& voxelSet);
void fillTrunkHole(voxelObject& voxelSet);
void finalizeVoxelObject(voxelObject& voxelSet);
void rotateModelinZAxis(voxelObject& voxelSet);
};