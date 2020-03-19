#include <k4a/k4a.hpp>

#include <iostream>
#include <vector>
#include <array>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Pixel.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"

using namespace std;
using namespace cv;
using namespace sen;

int main(int argc, char** argv)
{
	const uint32_t deviceCount = k4a::device::get_installed_count();
	std::cout << "current " << deviceCount << " kinect(s) are connected." << std::endl;
	if (deviceCount < 1)
	{
		cerr << "device list is empty." << std::endl;
		return -1;
	}
	
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;
	std::vector<k4a::device> sub_list(deviceCount);
	cout << "Started opening sub devices..." << endl;
	for (int i = 1; i < deviceCount; i++)
	{
		sub_list[i] = k4a::device::open(i);
		sub_list[i].start_cameras(&config);
		cout << "Opening sub device "<< i << endl;
	}	
	cout << "Started opening master device..." << endl;
	sub_list[0] = k4a::device::open(K4A_DEVICE_DEFAULT);
	sub_list[0].start_cameras(&config);

	cout << "Finished opening K4A device." << endl;

	std::vector<Pixel> depthTextureBuffer;
	std::vector<Pixel> irTextureBuffer;
	uint8_t *colorTextureBuffer;
	
	//device:0 master device:1-x sub
	std::vector<k4a::capture> capture_list(deviceCount);

	k4a::image depthImage;
	k4a::image colorImage;
	k4a::image irImage;

	cv::Mat depthFrame;
	cv::Mat colorFrame;
	cv::Mat irFrame;

	while (1)
	{
		bool capture_status = true;
		for (int i = 0; i < deviceCount; i++)
		{
			capture_status = capture_status && sub_list[i].get_capture(&capture_list[i], std::chrono::milliseconds(0));
		}
		
		if (capture_status)
		{
			for (int i = 0; i < deviceCount; i++)
			{
				depthImage = capture_list[i].get_depth_image();
				colorImage = capture_list[i].get_color_image();
				irImage = capture_list[i].get_ir_image();

				ColorizeDepthImage(depthImage, DepthPixelColorizer::ColorizeBlueToRed, GetDepthModeRange(config.depth_mode), &depthTextureBuffer);
				ColorizeDepthImage(irImage, DepthPixelColorizer::ColorizeGreyscale, GetIrLevels(K4A_DEPTH_MODE_PASSIVE_IR), &irTextureBuffer);
				colorTextureBuffer = colorImage.get_buffer();

				depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());
				colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
				irFrame = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_8UC4, irTextureBuffer.data());
				cv::imshow(std::to_string(i)+"_kinect depth map master", depthFrame);
				cv::imshow(std::to_string(i)+"_kinect depth map master", depthFrame);
				cv::imshow(std::to_string(i)+"_kinect color frame master", colorFrame);
				cv::imshow(std::to_string(i)+"_kinect ir frame master", irFrame);
			}			
		}
		if (waitKey(30) == 27 || waitKey(30) == 'q')
		{
			sub_list[0].close();
			break;
		}
	}
	return 0;
}