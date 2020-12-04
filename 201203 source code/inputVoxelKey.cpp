#include <thread>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>

#include "inputVoxelKey.h"
#include "voxelKeyBasic.h"


/* input a voxel set
[in] voxelObject& inputVoxelObject
[out] vector<voxelKey>& inputSet
*/
inputVoxelKey::inputVoxelKey(voxelObject& inputVoxelObject, vector<voxelKey>& inputSet):
	window("input voxel key set"),
	voxelSet(inputVoxelObject)
{
	cout << "preparing the input window please wait ... ";
	//register keyboard callbacks
	window.registerKeyboardCallback(&inputVoxelKey::keyboardEventOccurred, *this, nullptr);

	//reset camera
	window.setBackgroundColor(1.0, 1.0, 1.0);
	window.resetCameraViewpoint("objectVoxel");

	// name color
	orange.r = 1.0;
	orange.g = 0.5;
	orange.b = 0;
	green.r = 0.2;
	green.g = 1.0;
	green.b = 0.2;
	yellow.r = 1.0;
	yellow.g = 0.8;
	yellow.b = 0.2;
	blue.r = 0.2;
	blue.g = 0.8;
	blue.b = 1.0;
	black.r = 0;
	black.g = 0;
	black.b = 0;

	// display object voxels
	addVoxelKey(voxelSet.objectSet, 1, blue, false, "objectVoxel");
	showLegend();
	activeKey.x = 0;
	activeKey.y = 0;
	activeKey.z = 0;

	// run
	update();
	run();

	// store input result
	inputSet.clear();
	inputSet = inputKey;
	cout << "input total " << inputSet.size() << " voxels" << endl;
}


/* add a voxel set to the display rendering window
[in] vector<voxelKey>& set
[in] const int thickness
[in] const color& displayColor
[in] const bool wireFrame
[in] const string& id
*/
void inputVoxelKey::addVoxelKey(vector<voxelKey>& set, const int thickness, const color& displayColor, const bool wireFrame, const string& id)
{
	pcl::PointCloud<pcl::PointXYZ>::VectorType centroid;
	voxelKeyToCentroid(set, voxelSet.resolution, voxelSet.originPoint, centroid);

	vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();

	// Create every cubes to be displayed
	double s = voxelSet.resolution / 2.0;
	for (const auto& point : centroid)
	{
		double x = point.x;
		double y = point.y;
		double z = point.z;

		vtkSmartPointer<vtkCubeSource> wk_cubeSource = vtkSmartPointer<vtkCubeSource>::New();

		wk_cubeSource->SetBounds(x - s, x + s, y - s, y + s, z - s, z + s);
		wk_cubeSource->Update();

		appendFilter->AddInputData(wk_cubeSource->GetOutput());
	}

	// Remove any duplicate points
	vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();

	cleanFilter->SetInputConnection(appendFilter->GetOutputPort());
	cleanFilter->Update();

	//Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> multiMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	multiMapper->SetInputConnection(cleanFilter->GetOutputPort());

	vtkSmartPointer<vtkActor> multiActor = vtkSmartPointer<vtkActor>::New();
	multiActor->SetMapper(multiMapper);

	multiActor->GetProperty()->SetColor(displayColor.r, displayColor.g, displayColor.b);
	multiActor->GetProperty()->SetAmbient(1.0);
	multiActor->GetProperty()->SetLineWidth(thickness);
	multiActor->GetProperty()->EdgeVisibilityOn();
	multiActor->GetProperty()->SetOpacity(1.0);
	if (wireFrame)
	{
		multiActor->GetProperty()->SetRepresentationToWireframe();
	}
	else
	{
		multiActor->GetProperty()->SetRepresentationToSurface();
	}

	// Add the actor to the scene
	window.getRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(multiActor);
}


/* clear the inputKey in the rendering window
*/
void inputVoxelKey::clearInput()
{
	//remove cubes of all walls
	vtkRenderer* renderer = window.getRenderWindow()->GetRenderers()->GetFirstRenderer();
	while (renderer->GetActors()->GetNumberOfItems() > 1)
	{
		renderer->RemoveActor(renderer->GetActors()->GetLastActor());
	}
}


/* refresh the rendering
*/
void inputVoxelKey::update()
{
	clearInput();
	vector<voxelKey> set;
	set.push_back(activeKey);
	addVoxelKey(set, 2, orange, true, "activeKey");
	addVoxelKey(inputKey, 1, yellow, false, "inputKey");
}


/* detect keyboard press to adjust activeKey and inputKey
*/
void inputVoxelKey::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void*)
{
	if (event.getKeySym() == "Up" && event.keyDown())
	{
		activeKey.y++;
		update();
	}
	else if (event.getKeySym() == "Down" && event.keyDown())
	{
		activeKey.y--;
		update();
	}
	else if (event.getKeySym() == "Left" && event.keyDown())
	{
		activeKey.x--;
		update();
	}
	else if (event.getKeySym() == "Right" && event.keyDown())
	{
		activeKey.x++;
		update();
	}
	else if (event.getKeySym() == "Control_L" && event.keyDown())
	{
		activeKey.z++;
		update();
	}
	else if (event.getKeySym() == "Alt_L" && event.keyDown())
	{
		activeKey.z--;
		update();
	}
	else if (event.getKeySym() == "space" && event.keyDown())
	{
		if (belongsToSet(activeKey, inputKey))
		{
			vector<voxelKey> set;
			set.push_back(activeKey);
			setDifference(inputKey, set);
		}
		else
		{
			inputKey.push_back(activeKey);
		}
		update();
	}
}


/* show instrunction in the window
*/
void inputVoxelKey::showLegend()
{
	window.addText("Instruction", 20, 100, black.r, black.g, black.b, "line1");
	window.addText("Right / Left  - move the active voxel towards x+1/x-1 direction", 20, 80, black.r, black.g, black.b, "line2");
	window.addText("Up / Down  - move the active voxel towards y+1/y-1 direction", 20, 70, black.r, black.g, black.b, "line3");
	window.addText("Ctrl   /   Alt  - move the active voxel towards z+1/z-1 direction", 20, 60, black.r, black.g, black.b, "line4");
	window.addText("Space         - add / remove active key to / from input key set", 20, 50, black.r, black.g, black.b, "line5");
	window.addText("directly close this window to confirm your input", 20, 30, black.r, black.g, black.b, "line6");
}


/* start the rendering window
*/
void inputVoxelKey::run()
{
	// Render and interact
	window.getRenderWindow()->Render();

	cout << "close the viewer window to confirm input and continue" << endl;

	while (!window.wasStopped())
	{
		//main loop of the visualizer
		window.spinOnce(100);
		this_thread::sleep_for(100ms);
	}
}

