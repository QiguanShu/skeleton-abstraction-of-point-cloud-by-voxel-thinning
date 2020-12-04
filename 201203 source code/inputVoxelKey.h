#include <pcl/visualization/pcl_visualizer.h>
#include "voxelBasic.h"

class inputVoxelKey
{
public:
	// main function
	inputVoxelKey(voxelObject& inputVoxelObject, vector<voxelKey>& inputSet);
	struct color
	{
		float r;
		float g;
		float b;
	};
	
private:
	pcl::visualization::PCLVisualizer window;
	color yellow;
	color blue;
	color orange;
	color green;
	color black;
	// subfunctions
	void addVoxelKey(vector<voxelKey>& set, const int thickness, const color& displayColor, const bool wireFrame, const string& id);
	void clearInput();
	void update();
	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void*);
	void showLegend();
	void run();
	voxelObject voxelSet;
	voxelKey activeKey;
	vector<voxelKey> inputKey;
};