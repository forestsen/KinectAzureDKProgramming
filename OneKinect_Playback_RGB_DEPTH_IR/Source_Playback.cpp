#include <k4a/k4a.hpp>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <k4arecord/record.h>

#include <windows.h>

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <atomic>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Pixel.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"
#include "Util.h"

using namespace std;
using namespace cv;
using namespace sen;

int main(int argc, char* argv[])
{
	k4a_playback_t playback = NULL;
	k4a_result_t result;
	k4a_stream_result_t stream_result;

	string input_path = "recording.mkv";
	result = k4a_playback_open(input_path.c_str(), &playback);
	if (result != K4A_RESULT_SUCCEEDED || playback == NULL)
	{
		std::cout << "Failed to open recording: " << input_path << std::endl;
		return -1;
	}

	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_YUY2;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;

	k4a_capture_t capture = NULL;

	k4a_image_t depthImage = NULL;
	k4a_image_t compressed_color_image = NULL;
	k4a_image_t irImage = NULL;
	k4a_image_t uncompressed_color_image = NULL;

	cv::Mat depthFrame;
	cv::Mat colorFrame;
	cv::Mat irFrame;

	std::vector<Pixel> depthTextureBuffer;
	std::vector<Pixel> irTextureBuffer;
	uint8_t *colorTextureBuffer;

	while (1)
	{
		stream_result = k4a_playback_get_next_capture(playback, &capture);
		if (stream_result != K4A_STREAM_RESULT_SUCCEEDED || capture == NULL)
		{
			std::cout << "Failed to fetch frame." << std::endl;
			return -1;
		}

		depthImage = k4a_capture_get_depth_image(capture);
		compressed_color_image = k4a_capture_get_color_image(capture);
		int color_width, color_height;
		color_width = k4a_image_get_width_pixels(compressed_color_image);
		color_height = k4a_image_get_height_pixels(compressed_color_image);

		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
			color_width,
			color_height,
			color_width * 4 * (int)sizeof(uint8_t),
			&uncompressed_color_image))
		{
			std::cout << "Failed to create image buffer\n";
			return -1;
		}

		tjhandle tjHandle;
		tjHandle = tjInitDecompress();
		if (tjDecompress2(tjHandle,
			k4a_image_get_buffer(compressed_color_image),
			static_cast<unsigned long>(k4a_image_get_size(compressed_color_image)),
			k4a_image_get_buffer(uncompressed_color_image),
			color_width,
			0, // pitch
			color_height,
			TJPF_BGRA,
			TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
		{
			std::cout << "Failed to decompress color frame\n";
			if (tjDestroy(tjHandle))
			{
				std::cout << "Failed to destroy turboJPEG handle\n";
			}
			return -1;
		}
		if (tjDestroy(tjHandle))
		{
			std::cout << "Failed to destroy turboJPEG handle\n";
			return -1;
		}


		///////////////////////////////
		irImage = k4a_capture_get_ir_image(capture);

		ColorizeDepthImage(depthImage, DepthPixelColorizer::ColorizeBlueToRed, GetDepthModeRange(config.depth_mode), &depthTextureBuffer);
		ColorizeDepthImage(irImage, DepthPixelColorizer::ColorizeGreyscale, GetIrLevels(K4A_DEPTH_MODE_PASSIVE_IR), &irTextureBuffer);
		colorTextureBuffer = k4a_image_get_buffer(uncompressed_color_image);

		depthFrame = cv::Mat(k4a_image_get_height_pixels(depthImage), k4a_image_get_width_pixels(depthImage), CV_8UC4, depthTextureBuffer.data());
		colorFrame = cv::Mat(k4a_image_get_height_pixels(uncompressed_color_image), k4a_image_get_width_pixels(uncompressed_color_image), CV_8UC4, colorTextureBuffer);
		irFrame = cv::Mat(k4a_image_get_height_pixels(irImage), k4a_image_get_width_pixels(irImage), CV_8UC4, irTextureBuffer.data());
		cv::imshow("kinect depth map master", depthFrame);
		cv::imshow("kinect color frame master", colorFrame);
		cv::imshow("kinect ir frame master", irFrame);

		if (waitKey(30) == 27 || waitKey(30) == 'q')
		{
			k4a_playback_close(playback);
			break;
		}
	}
	k4a_image_release(depthImage);
	k4a_image_release(uncompressed_color_image);
	k4a_image_release(compressed_color_image);
	k4a_image_release(irImage);


	return 0;
}