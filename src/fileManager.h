#include <pcl/io/ply_io.h>
#include "voxelBasic.h"
using namespace std;

// for .ply file
bool readPly(string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
void writePly(string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

// for .vox file
bool readVox(string fileName, double& resolution, voxelObject& voxelSet);
void writeVox(voxelObject& voxelSet);

// for .skl file
void writeSkl(voxelObject& voxelSet);
bool readSkl(string fileName, double& resolution, voxelObject& voxelSet);

// for .bra file
void writeBra(voxelObject& voxelSet, vector<branch>& branches);

// for .reb file
void writeReb(voxelObject& voxelSet, vector<voxelKey>& commonSet, vector<voxelKey>& missingSet, vector<voxelKey>& redundantSet);