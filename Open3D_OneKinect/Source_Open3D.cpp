#include <iostream>
#include <string>
#include <memory>

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

#include <Open3D/IO/ClassIO/IJsonConvertibleIO.h>
#include <Open3D/IO/Sensor/AzureKinect/AzureKinectSensor.h>

#include <Open3D/Visualization/Visualizer/Visualizer.h>
#include <Open3D/Visualization/Visualizer/VisualizerWithKeyCallback.h>
#include <Open3D/Visualization/Utility/DrawGeometry.h>

#include <Open3D/Geometry/RGBDImage.h>
#include <Open3D/Geometry/PointCloud.h>

#include <windows.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

using namespace std;

int main(int argc, char **argv)
{
	open3d::io::AzureKinectSensorConfig azureKinectConfig;
	string config_filename = "azure_kinect.json";
	open3d::io::ReadIJsonConvertibleFromJSON(config_filename, azureKinectConfig);
	open3d::io::AzureKinectSensor azureKinect(azureKinectConfig);
	string intrinsic_config_filename = "azure_kinect_intrinsic.json";
	open3d::camera::PinholeCameraIntrinsic pinhole_camera;
	open3d::io::ReadIJsonConvertible(intrinsic_config_filename, pinhole_camera);

	azureKinect.Connect(0);

	bool flag_exit = false;
	bool is_geometry_added = false;
	open3d::visualization::VisualizerWithKeyCallback vis;
	vis.RegisterKeyCallback(GLFW_KEY_ESCAPE,
		[&](open3d::visualization::Visualizer *vis) {
		flag_exit = true;
		return false;
	});
	vis.CreateVisualizerWindow("Open3D Azure Kinect Recorder", 1280, 720);
	do {
		std::shared_ptr<open3d::geometry::RGBDImage> im_rgbd = azureKinect.CaptureFrame(true);
		
		if (im_rgbd == nullptr) 
		{
			std::cout << "Invalid capture, skipping this frame\n";
			continue;
		}

		//std::shared_ptr<open3d::geometry::PointCloud> pcd = open3d::geometry::PointCloud::CreateFromDepthImage(im_rgbd->depth_, pinhole_camera);

		if (!is_geometry_added) {
			vis.AddGeometry(im_rgbd);
			//vis.AddGeometry(pcd);
			is_geometry_added = true;
		}

		// Update visualizer
		vis.UpdateGeometry();
		vis.PollEvents();
		vis.UpdateRender();

	} while (!flag_exit);

	return 0;
}