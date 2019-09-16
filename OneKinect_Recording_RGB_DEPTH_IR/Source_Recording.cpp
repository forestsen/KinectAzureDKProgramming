#include <k4a/k4a.hpp>
#include <k4a/k4a.h>
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

k4a_device_configuration_t config;
k4a_device_t device;
k4a_capture_t capture;
atomic<bool> exiting(false);

int main(int argc, char* argv[])
{
	const uint32_t deviceCount = k4a::device::get_installed_count();
	if (deviceCount == 0)
	{
		cout << "no azure kinect devices detected!" << endl;
	}

	config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;

	cout << "Started opening K4A device..." << endl;
	k4a_device_open(K4A_DEVICE_DEFAULT, &device);
	k4a_device_start_cameras(device, &config);
	cout << "Finished opening K4A device." << endl;

	k4a_record_t recording;
	string file_name_recording = "recording.mkv";
	k4a_record_create(file_name_recording.c_str(), device, config, &recording);
	k4a_record_write_header(recording);

	int32_t timeout_sec_for_first_capture = 60;
	if (config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE)
	{
		timeout_sec_for_first_capture = 360;
		std::cout << "[subordinate mode] Waiting for signal from master" << std::endl;
	}
	clock_t first_capture_start = clock();
	k4a_wait_result_t result = K4A_WAIT_RESULT_TIMEOUT;
	while (!exiting && (clock() - first_capture_start) < (CLOCKS_PER_SEC * timeout_sec_for_first_capture))
	{
		result = k4a_device_get_capture(device, &capture, 100);
		if (result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			k4a_capture_release(capture);
			break;
		}
		else if (result == K4A_WAIT_RESULT_FAILED)
		{
			std::cerr << "Runtime error: k4a_device_get_capture() returned error: " << result << std::endl;
			return 1;
		}
	}

	if (exiting)
	{
		k4a_device_close(device);
		return 0;
	}
	else if (result == K4A_WAIT_RESULT_TIMEOUT)
	{
		std::cerr << "Timed out waiting for first capture." << std::endl;
		return 1;
	}

	std::cout << "Started recording" << std::endl;

	int recording_length = 10;
	if (recording_length <= 0)
	{
		std::cout << "Press Ctrl-C to stop recording." << std::endl;
	}

	uint32_t camera_fps = k4a_convert_fps_to_uint(config.camera_fps);

	if (camera_fps <= 0 || (config.color_resolution == K4A_COLOR_RESOLUTION_OFF &&
		config.depth_mode == K4A_DEPTH_MODE_OFF))
	{
		std::cerr << "Either the color or depth modes must be enabled to record." << std::endl;
		return 1;
	}
	clock_t recording_start = clock();
	int32_t timeout_ms = 1000 / camera_fps;
	do
	{
		result = k4a_device_get_capture(device, &capture, timeout_ms);
		if (result == K4A_WAIT_RESULT_TIMEOUT)
		{
			continue;
		}
		else if (result != K4A_WAIT_RESULT_SUCCEEDED)
		{
			std::cerr << "Runtime error: k4a_device_get_capture() returned " << result << std::endl;
			break;
		}
		k4a_record_write_capture(recording, capture);
		k4a_capture_release(capture);
	} while (!exiting && result != K4A_WAIT_RESULT_FAILED &&
		(recording_length < 0 || (clock() - recording_start < recording_length * CLOCKS_PER_SEC)));

	if (!exiting)
	{
		exiting = true;
		std::cout << "Stopping recording..." << std::endl;
	}

	k4a_device_stop_cameras(device);

	std::cout << "Saving recording..." << std::endl;
	k4a_record_flush(recording);
	k4a_record_close(recording);

	std::cout << "Done" << std::endl;
	k4a_device_close(device);

	return 0;
}