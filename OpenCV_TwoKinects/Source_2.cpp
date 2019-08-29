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

int main(int argc, char **argv)
{
	const uint32_t deviceCount = k4a::device::get_installed_count();
	if (deviceCount == 0)
	{
		cout << "no azure kinect devices detected!" << endl;
	}

	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;

	cout << "Started opening K4A device..." << endl;
	k4a::device dev_sub = k4a::device::open(1);
	dev_sub.start_cameras(&config);
	k4a::device dev_master = k4a::device::open(K4A_DEVICE_DEFAULT);
	dev_master.start_cameras(&config);
	cout << "Finished opening K4A device." << endl;

	std::vector<Pixel> depthTextureBuffer;
	std::vector<Pixel> irTextureBuffer;
	uint8_t *colorTextureBuffer;

	k4a::capture capture_master;
	k4a::capture capture_sub;

	k4a::image depthImage;
	k4a::image colorImage;
	k4a::image irImage;

	cv::Mat depthFrame;
	cv::Mat colorFrame;
	cv::Mat irFrame;

	while (1)
	{
		if (dev_master.get_capture(&capture_master, std::chrono::milliseconds(0)) && dev_sub.get_capture(&capture_sub, std::chrono::milliseconds(0)))
		{
			{
				depthImage = capture_master.get_depth_image();
				colorImage = capture_master.get_color_image();
				irImage = capture_master.get_ir_image();

				ColorizeDepthImage(depthImage, DepthPixelColorizer::ColorizeBlueToRed, GetDepthModeRange(config.depth_mode), &depthTextureBuffer);
				ColorizeDepthImage(irImage, DepthPixelColorizer::ColorizeGreyscale, GetIrLevels(K4A_DEPTH_MODE_PASSIVE_IR), &irTextureBuffer);
				colorTextureBuffer = colorImage.get_buffer();

				depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());
				colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
				irFrame = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_8UC4, irTextureBuffer.data());
				cv::imshow("kinect depth map master", depthFrame);
				cv::imshow("kinect color frame master", colorFrame);
				cv::imshow("kinect ir frame master", irFrame);
			}

			{
				depthImage = capture_sub.get_depth_image();
				colorImage = capture_sub.get_color_image();
				irImage = capture_sub.get_ir_image();

				ColorizeDepthImage(depthImage, DepthPixelColorizer::ColorizeBlueToRed, GetDepthModeRange(config.depth_mode), &depthTextureBuffer);
				ColorizeDepthImage(irImage, DepthPixelColorizer::ColorizeGreyscale, GetIrLevels(K4A_DEPTH_MODE_PASSIVE_IR), &irTextureBuffer);
				colorTextureBuffer = colorImage.get_buffer();

				depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());
				colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
				irFrame = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_8UC4, irTextureBuffer.data());
				cv::imshow("kinect depth map sub", depthFrame);
				cv::imshow("kinect color frame sub", colorFrame);
				cv::imshow("kinect ir frame sub", irFrame);
			}
		}
		if (waitKey(30) == 27 || waitKey(30) == 'q')
		{
			dev_master.close();
			break;
		}
	}
	return 0;
}