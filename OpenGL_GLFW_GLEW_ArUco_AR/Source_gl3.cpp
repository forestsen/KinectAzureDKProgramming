#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <thread>
#include <mutex>

#include <windows.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <k4a/k4a.hpp>

#include <aruco/aruco.h>

#include <Pixel.h>
#include <Util.h>
#include <DepthPixelColorizer.h>
#include <StaticImageProperties.h>

#include <Shader.h>
#include <Texture.h>
#include <ARRenderer.h>
#include <VideoRenderer.h>

using namespace std;
using namespace cv;
using namespace glm;
using namespace sen;

GLFWwindow* window;

int bufferIndex = 0;
k4a::image initTexture;
k4a::image colorImageBuffer;
std::array<k4a::image, 30> colorBuffer;
k4a::image outColorFrame;

std::thread capturing_thread;
std::mutex mtx;

k4a::device device;
k4a::capture capture;

aruco::Dictionary dic;
Mat generatedMarker;
aruco::CameraParameters CamParam;
aruco::MarkerDetector MDetector;
std::map<int, aruco::MarkerPoseTracker> MTracker;
float MarkerSize = 0.1f;
double proj_matrix[16];
double modelview_matrix[16];

VideoRenderer videoRenderer;
ARRenderer arRenderer;

void arucoSetup(const k4a::calibration calibration, const int &texture_width, const int &texture_height, cv::Mat &camera_matrix_color, cv::Mat &dist_coeffs_color, aruco::CameraParameters &CamParam)
{
	k4a_calibration_intrinsic_parameters_t intrinsics_color = calibration.color_camera_calibration.intrinsics.parameters;
	vector<float> _camera_matrix = {
	   intrinsics_color.param.fx, 0.f, intrinsics_color.param.cx,
	   0.f, intrinsics_color.param.fy, intrinsics_color.param.cy,
	   0.f, 0.f, 1.f };
	camera_matrix_color = Mat(3, 3, CV_32F, &_camera_matrix[0]);
	vector<float> _dist_coeffs = { intrinsics_color.param.k1, intrinsics_color.param.k2, intrinsics_color.param.p1,
								   intrinsics_color.param.p2, intrinsics_color.param.k3, intrinsics_color.param.k4,
								   intrinsics_color.param.k5, intrinsics_color.param.k6 };
	dist_coeffs_color = Mat::zeros(5, 1, CV_32F);

	{
		dic = aruco::Dictionary::load("ARUCO_MIP_36h12");
		CamParam.setParams(camera_matrix_color, dist_coeffs_color, cv::Size(texture_width, texture_height));
		MDetector.setDictionary("ARUCO_MIP_36h12", 0.05f);
		MDetector.setDetectionMode(aruco::DM_VIDEO_FAST);
	}
}
k4a_device_configuration_t k4aSetup()
{
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;
	return config;
}
void frameRetriever()
{
	while (1)
	{
		if (device.get_capture(&capture, std::chrono::milliseconds(0)))
		{
			colorImageBuffer = capture.get_color_image();
			cv::Mat colorFrame = cv::Mat(colorImageBuffer.get_height_pixels(), colorImageBuffer.get_width_pixels(), CV_8UC4, colorImageBuffer.get_buffer());

			cvtColor(colorFrame, colorFrame, COLOR_BGRA2BGR);

			{
				vector<aruco::Marker> Markers = MDetector.detect(colorFrame);
				for (auto &marker : Markers)
				{
					MTracker[marker.id].estimatePose(marker, CamParam, MarkerSize);
				}
				if (CamParam.isValid() && MarkerSize != -1)
				{
					for (unsigned int i = 0; i < Markers.size(); ++i)
					{
						if (Markers[i].isPoseValid())
						{
							Markers[i].glGetModelViewMatrix(modelview_matrix);
						}
					}
				}
			}
		}

		if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(window))
		{
			break;
		}

		mtx.lock();
		colorBuffer[(bufferIndex++) % 30] = colorImageBuffer;
		mtx.unlock();
	}
}
void reshape(GLFWwindow *window, int w, int h)
{
	glViewport(0, 0, w, h);
	CamParam.glGetProjectionMatrix(cv::Size(initTexture.get_width_pixels(), initTexture.get_height_pixels()), cv::Size(w, h), proj_matrix, 0.05, 10);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}
void drawBackground()
{
	if (bufferIndex > 0)
	{
		mtx.lock();
		outColorFrame = colorBuffer[(bufferIndex - 1) % 30];
		mtx.unlock();

		videoRenderer.render(outColorFrame);
	}
}
void drawAugmentedScene()
{
	arRenderer.render(proj_matrix, modelview_matrix);
}
void display()
{
	drawBackground();
	drawAugmentedScene();
}

int main(int argc, char **argv)
{
	const uint32_t deviceCount = k4a::device::get_installed_count();
	if (deviceCount == 0)
	{
		cout << "no azure kinect devices detected!" << endl;
	}

	k4a_device_configuration_t config = k4aSetup();

	cout << "Started opening K4A device..." << endl;
	device = k4a::device::open(0);
	device.start_cameras(&config);
	cout << "Finished opening K4A device." << endl;

	while (1)
	{
		if (device.get_capture(&capture, std::chrono::milliseconds(0)))
		{
			initTexture = capture.get_color_image();
			break;
		}
	}

	int texture_width = initTexture.get_width_pixels();
	int texture_height = initTexture.get_height_pixels();

	k4a::calibration calibration = device.get_calibration(config.depth_mode, config.color_resolution);
	Mat camera_matrix_color;
	Mat dist_coeffs_color;
	arucoSetup(calibration, texture_width, texture_height, camera_matrix_color, dist_coeffs_color, CamParam);
	CamParam.glGetProjectionMatrix(cv::Size(texture_width, texture_height), cv::Size(texture_width, texture_width), proj_matrix, 0.05, 10);

	//***********************************************************************************************************

	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	window = glfwCreateWindow(texture_width, texture_height, "GL3 AR", NULL, NULL);
	if (window == NULL)
	{
		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible.\n");
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	glewExperimental = true;
	if (glewInit() != GLEW_OK)
	{
		fprintf(stderr, "Failed to initialize GLEW\n");
		return -1;
	}

	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
	glfwSetFramebufferSizeCallback(window, reshape);

	//***********************************************************************************************************

	glViewport(0, 0, texture_width, texture_height);
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	videoRenderer.setup("../../TransformVertexShader.vertexshader", "../../TextureFragmentShader.fragmentshader");
	videoRenderer.initTexture(initTexture);

	arRenderer.setup("../../TransformVertexShader_AR.vertexshader", "../../ColorFragmentShader.fragmentshader");

	capturing_thread = std::thread(frameRetriever);

	do
	{
		display();

		glfwSwapBuffers(window);
		glfwPollEvents();

	} while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && !glfwWindowShouldClose(window));

	glDeleteVertexArrays(1, &VertexArrayID);

	videoRenderer.deleteBuffer();
	arRenderer.deleteBuffer();

	glfwTerminate();

	capturing_thread.join();
	device.close();

	return 0;
}