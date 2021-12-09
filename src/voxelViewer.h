#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include "voxelBasic.h"

struct color
{
	float r;
	float g;
	float b;
};

class voxelViewer
{
public:

	// main function
	voxelViewer(voxelObject& inputVoxelSet);
	bool showCloud;

	// subfunctions
	void addVoxelKey(vector<voxelKey>& set, const int thickness, const color& displayColor, const bool wireFrame, const string& id);
	void run();

private:
	pcl::visualization::PCLVisualizer window;
	void clearView();
	void update();
	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void*);
	void showLegend();
	voxelObject voxelSet;
};