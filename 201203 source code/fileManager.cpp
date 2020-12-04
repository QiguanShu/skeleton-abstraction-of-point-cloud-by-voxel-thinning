#include <pcl/common/time.h>

#include "fileManager.h"


/* load .ply data to PointXYZRGB cloud
[in] string fileName
[out] pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
*/
bool readPly(string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	cout << "\nloading file: " << fileName << "... ";
	pcl::StopWatch timer;
	pcl::PLYReader reader;
	
	if(reader.read(fileName, *cloud) == 0)
	{
		cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
		cout << cloud->width * cloud->height << " points has been loaded" << endl;
		return true;
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... file not found" << endl;
	return false;
}


/* write PointXYZRGB cloud to .ply file
[in] string fileName
[out] pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
*/
void writePly(string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	cout << "\nsaving file: " << fileName << "... ";
	pcl::StopWatch timer;
	pcl::PLYWriter writer;
	writer.write(fileName, *cloud);
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << cloud->width * cloud->height << " points has been written to " << fileName << endl;
}



/* read voxel data from .vox file to voxel object
[in] string fileName
[out] voxelObject& voxelSet
[return] boolean values
*/
bool readVox(string fileName, double& resolution, voxelObject& voxelSet)
{
	fileName.replace(fileName.find(".ply"), sizeof(".ply") - 1, "_" + to_string(resolution) + ".vox");
	ifstream voxelFile;
	voxelFile.open(fileName);
	if (voxelFile.is_open())
	{
		cout << "\nload voxel data file: " << fileName << "... ";
		pcl::StopWatch timer;
		voxelFile.seekg(11, ios::cur); // move cursor after "file name: "
		voxelFile >> voxelSet.fileName;
		voxelFile.ignore(); // jump cursor to the next line
		voxelFile.seekg(20, ios::cur); // move cursor after "origin point x y z: "
		voxelFile >> voxelSet.originPoint.x >> voxelSet.originPoint.y >> voxelSet.originPoint.z;
		voxelFile.ignore(); // jump cursor to the next line
		voxelFile.seekg(12, ios::cur); // move cursor after "resolution: "
		voxelFile >> voxelSet.resolution;
		voxelFile.ignore(); // jump cursor to the next line
		voxelFile.seekg(20, ios::cur); // move cursor after "boundary key x y z: "
		voxelFile >> voxelSet.boundaryKey.x >> voxelSet.boundaryKey.y >> voxelSet.boundaryKey.z;
		voxelFile.ignore(); // jump cursor to the next line
		voxelFile.seekg(11, ios::cur); // move cursor after "voxel keys:"
		voxelFile.ignore(); // jump cursor to the next line
		voxelSet.objectSet.clear();
		int x, y, z;
		while (voxelFile >> x >> y >> z)
		{
			voxelKey key;
			key.x = x;
			key.y = y;
			key.z = z;
			voxelSet.objectSet.push_back(key);
		}
		voxelFile.close();
		cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
		cout << voxelSet.objectSet.size() << " voxels have been loaded" << endl;
		cout << "origin point: (" << voxelSet.originPoint.x << ", " << voxelSet.originPoint.y << ", " << voxelSet.originPoint.z << ")" << endl;
		cout << "boundary key: (" << voxelSet.boundaryKey.x << ", " << voxelSet.boundaryKey.y << ", " << voxelSet.boundaryKey.z << ")" << endl;
		return true;
	}
	voxelFile.close();
	return false;
}


/* save voxel data to .vox file
[in] voxelObject& voxelSet
*/
void writeVox(voxelObject& voxelSet)
{
	string saveName = voxelSet.fileName;
	saveName.replace(saveName.find(".ply"), sizeof(".ply") - 1, "_" + to_string(voxelSet.resolution) + ".vox");
	cout << "\nsave voxel data file: " << saveName << "... ";
	pcl::StopWatch timer;
	ofstream saveVoxel(saveName);
	saveVoxel << "file name: " << voxelSet.fileName << "\n";
	saveVoxel << "origin point x y z: " << voxelSet.originPoint.x << " " << voxelSet.originPoint.y << " " << voxelSet.originPoint.z << "\n";
	saveVoxel << "resolution: " << voxelSet.resolution << "\n";
	saveVoxel << "boundary key x y z: " << voxelSet.boundaryKey.x << " " << voxelSet.boundaryKey.y << " " << voxelSet.boundaryKey.z << "\n";
	saveVoxel << "voxel keys:\n";
	for (voxelKey key : voxelSet.objectSet)
	{
		saveVoxel << key.x << " " << key.y << " " << key.z << "\n";
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << voxelSet.objectSet.size() << " voxels have been saved to: " << saveName << endl;
}


/* save skeleton data to .skl file
[in] voxelObject& voxelSet
*/
void writeSkl(voxelObject& voxelSet)
{
	string saveName = voxelSet.fileName;
	saveName.replace(saveName.find(".ply"), sizeof(".ply") - 1, "_" + to_string(voxelSet.resolution) + ".skl");
	cout << "\nsave voxel data file: " << saveName << "... ";
	pcl::StopWatch timer;
	ofstream saveVoxel(saveName);
	saveVoxel << "file name: " << voxelSet.fileName << "\n";
	saveVoxel << "origin point x y z: " << voxelSet.originPoint.x << " " << voxelSet.originPoint.y << " " << voxelSet.originPoint.z << "\n";
	saveVoxel << "resolution: " << voxelSet.resolution << "\n";
	saveVoxel << "boundary key x y z: " << voxelSet.boundaryKey.x << " " << voxelSet.boundaryKey.y << " " << voxelSet.boundaryKey.z << "\n";
	saveVoxel << "total iteration: " << voxelSet.totalIteration << "\n";
	saveVoxel << "voxel keys x y z exp0 exp1 exp2 exp3 exp4:\n";
	for (voxelKey key : voxelSet.objectSet)
	{
		saveVoxel << key.x << " " << key.y << " " << key.z << " " << key.exposureIteration[0] << " " << key.exposureIteration[1] << " " << key.exposureIteration[2] << " " << key.exposureIteration[3] << " " << key.exposureIteration[4] << "\n";
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << voxelSet.objectSet.size() << " voxels have been saved to: " << saveName << endl;
}


/* read skeleton data from .skl file to voxel object
[in] string fileName
[out] voxelObject voxelSet
[out] int& totalIteration
[return] boolean values
*/
bool readSkl(string fileName, double& resolution, voxelObject& voxelSet)
{
	fileName.replace(fileName.find(".ply"), sizeof(".ply") - 1, "_" + to_string(resolution) + ".skl");
	ifstream voxelFile;
	voxelFile.open(fileName);
	if (voxelFile.is_open())
	{
		cout << "\nload voxel data file: " << fileName << "... ";
		pcl::StopWatch timer;
		voxelFile.seekg(11, ios::cur); // move cursor after "file name: "
		voxelFile >> voxelSet.fileName;
		voxelFile.ignore(); // jump cursor to the next line
		voxelFile.seekg(20, ios::cur); // move cursor after "origin point x y z: "
		voxelFile >> voxelSet.originPoint.x >> voxelSet.originPoint.y >> voxelSet.originPoint.z;
		voxelFile.ignore(); // jump cursor to the next line
		voxelFile.seekg(12, ios::cur); // move cursor after "resolution: "
		voxelFile >> voxelSet.resolution;
		voxelFile.ignore(); // jump cursor to the next line
		voxelFile.seekg(20, ios::cur); // move cursor after "boundary key x y z: "
		voxelFile >> voxelSet.boundaryKey.x >> voxelSet.boundaryKey.y >> voxelSet.boundaryKey.z;
		voxelFile.ignore(); // jump cursor to the next line
		voxelFile.seekg(17, ios::cur); // move cursor after "total iteration: "
		voxelFile >> voxelSet.totalIteration;
		voxelFile.ignore(); // jump cursor to the next line
		voxelFile.seekg(42, ios::cur); // move cursor after "voxel keys:"
		voxelFile.ignore(); // jump cursor to the next line
		voxelSet.objectSet.clear();
		int x, y, z;
		int exp[5];
		while (voxelFile >> x >> y >> z >> exp[0] >> exp[1] >> exp[2] >> exp[3] >> exp[4])
		{
			voxelKey key;
			key.x = x;
			key.y = y;
			key.z = z;
			for(int i=0; i < 5; i++)
			{
				key.exposureIteration[i] = exp[i];
			}
			voxelSet.objectSet.push_back(key);
		}
		voxelFile.close();
		cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
		cout << voxelSet.objectSet.size() << " voxels have been loaded" << endl;
		cout << "origin point: (" << voxelSet.originPoint.x << ", " << voxelSet.originPoint.y << ", " << voxelSet.originPoint.z << ")" << endl;
		cout << "boundary key: (" << voxelSet.boundaryKey.x << ", " << voxelSet.boundaryKey.y << ", " << voxelSet.boundaryKey.z << ")" << endl;
		return true;
	}
	voxelFile.close();
	return false;
}


/* save branch data to .bra file
[in] voxelObject& voxelSet
[in] vector<vector<voxelKey>>& branches
*/
void writeBra(voxelObject& voxelSet, vector<branch>& branches)
{
	string saveName = voxelSet.fileName;
	saveName.replace(saveName.find(".ply"), sizeof(".ply") - 1, "_" + to_string(voxelSet.resolution) + ".bra");
	cout << "\nsave branch data file: " << saveName << "... ";
	pcl::StopWatch timer;
	ofstream saveVoxel(saveName);
	saveVoxel << "file name: " << voxelSet.fileName << "\n";
	saveVoxel << "origin point x y z: " << voxelSet.originPoint.x << " " << voxelSet.originPoint.y << " " << voxelSet.originPoint.z << "\n";
	saveVoxel << "resolution: " << voxelSet.resolution << "\n";
	int branchCount = 0;
	for(branch oneBranch : branches)
	{
		branchCount++;
		saveVoxel << "# " << branchCount << " th branch with average voxel radius - " << oneBranch.voxelRadius << " elliptical ratio - " << oneBranch.ellipticalRatio << " x y z:\n";
		for(voxelKey key : oneBranch.branckKey)
		{
			saveVoxel << key.x << " " << key.y << " " << key.z << "\n";
		}
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << branchCount << " branches have been saved to: " << saveName << endl;
}


/* save voxel data to .reb file
[in] voxelObject& voxelSet
[in] vector<voxelKey>& commonSet
[in] vector<voxelKey>& missingSet
[in] vector<voxelKey>& redundantSet
*/
void writeReb(voxelObject& voxelSet, vector<voxelKey>& commonSet, vector<voxelKey>& missingSet, vector<voxelKey>& redundantSet)
{
	string saveName = voxelSet.fileName;
	saveName.replace(saveName.find(".ply"), sizeof(".ply") - 1, "_" + to_string(voxelSet.resolution) + ".reb");
	cout << "\nsave voxel data file: " << saveName << "... ";
	pcl::StopWatch timer;
	ofstream saveVoxel(saveName);
	saveVoxel << "file name: " << voxelSet.fileName << "\n";
	saveVoxel << "origin point x y z: " << voxelSet.originPoint.x << " " << voxelSet.originPoint.y << " " << voxelSet.originPoint.z << "\n";
	saveVoxel << "resolution: " << voxelSet.resolution << "\n";
	saveVoxel << "boundary key x y z: " << voxelSet.boundaryKey.x << " " << voxelSet.boundaryKey.y << " " << voxelSet.boundaryKey.z << "\n";
	saveVoxel << "# common voxel keys x y z:\n";
	for (voxelKey key : commonSet)
	{
		saveVoxel << key.x << " " << key.y << " " << key.z << "\n";
	}
	saveVoxel << "# missing voxel keys x y z:\n";
	for (voxelKey key : missingSet)
	{
		saveVoxel << key.x << " " << key.y << " " << key.z << "\n";
	}
	saveVoxel << "# redundant voxel keys x y z:\n";
	for (voxelKey key : redundantSet)
	{
		saveVoxel << key.x << " " << key.y << " " << key.z << "\n";
	}
	cout << "taking " << timer.getTimeSeconds() << " seconds ... completed" << endl;
	cout << voxelSet.objectSet.size() << " voxels have been saved to: " << saveName << endl;
}

