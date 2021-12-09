#include <thread>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>

#include "voxelViewer.h"


/* view voxel model
[in] voxelModel& inputVoxelSet
*/
voxelViewer::voxelViewer(voxelObject& inputVoxelSet):
	window("voxel viewer"),
	voxelSet(inputVoxelSet),
	showCloud(false)
{
	cout << "preparing the view window please wait ... ";
	//register keyboard callbacks
	window.registerKeyboardCallback(&voxelViewer::keyboardEventOccurred, *this, nullptr);

	//reset camera
	window.setBackgroundColor(1.0, 1.0, 1.0);
	window.resetCameraViewpoint("objectVoxel");
}


/* add a voxel set to the display rendering window
[in] vector<voxelKey>& set
[in] const int thickness
[in] const color& displayColor
[in] const bool wireFrame
[in] const string& id
*/
void voxelViewer::addVoxelKey(vector<voxelKey>& set, const int thickness, const color& displayColor, const bool wireFrame, const string& id)
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
void voxelViewer::clearView()
{
	//remove cubes of all walls
	vtkRenderer* renderer = window.getRenderWindow()->GetRenderers()->GetFirstRenderer();
	while (renderer->GetActors()->GetNumberOfItems() > 0)
	{
		renderer->RemoveActor(renderer->GetActors()->GetLastActor());
	}
}


/* refresh the rendering
*/
void voxelViewer::update()
{
	if(showCloud)
	{
		//show initialized point cloud
		int pointSize = 1;
		window.addPointCloud(voxelSet.cloud, "cloud");
		window.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud");
	}
	else
	{
		window.removePointCloud("cloud");
	}
}


/* detect keyboard press to adjust activeKey and inputKey
*/
void voxelViewer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void*)
{
	if (event.getKeySym() == "space" && event.keyDown())
	{
		showCloud = !showCloud;
		update();
	}
}


/* show instrunction in the window
*/
void voxelViewer::showLegend()
{
}


/* start the rendering window
*/
void voxelViewer::run()
{
	// Render and interact
	window.getRenderWindow()->Render();

	cout << "close the viewer window to continue" << endl;

	while (!window.wasStopped())
	{
		//main loop of the visualizer
		window.spinOnce(100);
		this_thread::sleep_for(100ms);
	}
}

