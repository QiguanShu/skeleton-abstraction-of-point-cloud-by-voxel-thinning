#include <thread>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>

#include "inputBoundaryBox.h"


/* input a boundary box for the model
[in] voxelObject& inputVoxelObject
[out] voxelKey& minBoundary
[out] voxelKey& maxBoundary
*/
inputBoundaryBox::inputBoundaryBox(voxelObject& inputVoxelObject, voxelKey& minBoundary, voxelKey& maxBoundary):
	window("boundary box setting"),
	voxelSet(inputVoxelObject),
	minKey(minBoundary),
	maxKey(maxBoundary),
	wallId(0)
{
	cout << "preparing the input window please wait ... ";

	//register keyboard callbacks
	window.registerKeyboardCallback(&inputBoundaryBox::keyboardEventOccurred, *this, nullptr);

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

	// run
	update();
	run();

	// store input result
	minBoundary = minKey;
	maxBoundary = maxKey;
	cout << "input boundary box from (" << minBoundary.x << ", " << minBoundary.y << ", " << minBoundary.z << ") to (" << maxBoundary.x << ", " << maxBoundary.y << ", " << maxBoundary.z << ")" << endl;
}


/* add a voxel set to the display rendering window
[in] vector<voxelKey>& set
[in] const int thickness
[in] const color& displayColor
[in] const bool wireFrame
[in] const string& id
*/
void inputBoundaryBox::addVoxelKey(vector<voxelKey>& set, const int thickness, const color& displayColor, const bool wireFrame, const string& id)
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


/* clear the wall cubes in the rendering window
*/
void inputBoundaryBox::clearWall()
{
	//remove cubes of all walls
	vtkRenderer* renderer = window.getRenderWindow()->GetRenderers()->GetFirstRenderer();
	while (renderer->GetActors()->GetNumberOfItems() > 1)
	{
		renderer->RemoveActor(renderer->GetActors()->GetLastActor());
	}
}


/* generate activeWall set and restWall set
*/
void inputBoundaryBox::setWall()
{
	activeWall.clear();
	restWall.clear();
	for (int x = minKey.x; x <= maxKey.x; x++)
	{
		for (int y = minKey.y; y <= maxKey.y; y++)
		{
			for (int z = minKey.z; z <= maxKey.z; z++) 
			{
				int sameXYZ = 0;
				if (x == minKey.x || x == maxKey.x)
				{
					sameXYZ++;
				}
				if (y == minKey.y || y == maxKey.y)
				{
					sameXYZ++;
				}
				if (z == minKey.z || z == maxKey.z)
				{
					sameXYZ++;
				}
				if(sameXYZ == 2 || sameXYZ == 3)
				{
					voxelKey key;
					key.x = x;
					key.y = y;
					key.z = z;
					switch (wallId)
					{
					case 0: // down side wall with min z
						if (z == minKey.z)
						{
							activeWall.push_back(key);
						}
						else
						{
							restWall.push_back(key);
						}
						break;
					case 1: // up side wall with max z
						if (z == maxKey.z)
						{
							activeWall.push_back(key);
						}
						else
						{
							restWall.push_back(key);
						}
						break;
					case 2: // left side wall with min x
						if (x == minKey.x)
						{
							activeWall.push_back(key);
						}
						else
						{
							restWall.push_back(key);
						}
						break;
					case 3: // right side wall with max x
						if (x == maxKey.x)
						{
							activeWall.push_back(key);
						}
						else
						{
							restWall.push_back(key);
						}
						break;
					case 4: // front side wall with min y
						if (y == minKey.y)
						{
							activeWall.push_back(key);
						}
						else
						{
							restWall.push_back(key);
						}
						break;
					case 5: // back side wall with max y
						if (y == maxKey.y)
						{
							activeWall.push_back(key);
						}
						else
						{
							restWall.push_back(key);
						}
						break;
					}
				}
			}
		}
	}
}


/* refresh the rendering
*/
void inputBoundaryBox::update()
{
	clearWall();
	setWall();
	addVoxelKey(activeWall, 2, orange, true, "activeWall");
	addVoxelKey(restWall, 1, green, true, "restWall");
}


/* detect keyboard press to adjust boundary values
*/
void inputBoundaryBox::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void*)
{
	if (event.getKeySym() == "Up" && event.keyDown())
	{
		if (wallId < 5)
		{
			wallId++;
		}else
		{
			wallId = 0;
		}
		update();
	}
	else if (event.getKeySym() == "Down" && event.keyDown())
	{
		if (wallId > 0)
		{
			wallId--;
		}
		else
		{
			wallId = 5;
		}
		update();
	}
	else if (event.getKeySym() == "Left" && event.keyDown())
	{
		switch(wallId)
		{
		case 0: // down side wall with min z
			if (minKey.z > 0) 
			{
				minKey.z--;
			}
			update();
			break;
		case 1: // up side wall with max z
			if (maxKey.z > minKey.z + 1)
			{
				maxKey.z--;
			}
			update();
			break;
		case 2: // left side wall with min x
			if (minKey.x > 0)
			{
				minKey.x--;
			}
			update();
			break;
		case 3: // right side wall with max x
			if (maxKey.x > minKey.x + 1)
			{
				maxKey.x--;
			}
			update();
			break;
		case 4: // front side wall with min y
			if (minKey.y > 0)
			{
				minKey.y--;
			}
			update();
			break;
		case 5: // back side wall with max y
			if (maxKey.y > minKey.y + 1)
			{
				maxKey.y--;
			}
			update();
			break;
		}
	}
	else if (event.getKeySym() == "Right" && event.keyDown())
	{
		switch (wallId)
		{
		case 0: // down side wall with min z
			if (minKey.z < maxKey.z - 1)
			{
				minKey.z++;
			}
			update();
			break;
		case 1: // up side wall with max z
			if (maxKey.z < voxelSet.boundaryKey.z)
			{
				maxKey.z++;
			}
			update();
			break;
		case 2: // left side wall with min x
			if (minKey.x < maxKey.x - 1)
			{
				minKey.x++;
			}
			update();
			break;
		case 3: // right side wall with max x
			if (maxKey.x < voxelSet.boundaryKey.x)
			{
				maxKey.x++;
			}
			update();
			break;
		case 4: // front side wall with min y
			if (minKey.y < maxKey.y - 1)
			{
				minKey.y++;
			}
			update();
			break;
		case 5: // back side wall with max y
			if (maxKey.y < voxelSet.boundaryKey.y)
			{
				maxKey.y++;
			}
			update();
			break;
		}
	}
}


/* show instrunction in the window
*/
void inputBoundaryBox::showLegend()
{
	window.addText("Instruction", 20, 100, black.r, black.g, black.b, "line1");
	window.addText("Up / Down - switch the selecting boundary walls", 20, 80, black.r, black.g, black.b, "line2");
	window.addText("Left    - move the selecting wall towards x-1/y-1/z-1 direction", 20, 70, black.r, black.g, black.b, "line3");
	window.addText("Right - move the selecting wall towards x+1/y+1/z+1 direction", 20, 60, black.r, black.g, black.b, "line4");
	window.addText("boundary walls represent the maximum calculating space including both object and void voxels", 20, 40, black.r, black.g, black.b, "line5");
	window.addText("directly close this window to confirm your input", 20, 30, black.r, black.g, black.b, "line6");
}


/* start the rendering window
*/
void inputBoundaryBox::run()
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

