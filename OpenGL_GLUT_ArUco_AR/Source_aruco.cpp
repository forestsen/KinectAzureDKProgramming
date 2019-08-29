#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <thread>
#include <mutex>

#include <windows.h>
#include <GL/freeglut.h>

#include <k4a/k4a.hpp>

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <aruco/aruco.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Pixel.h"
#include "Util.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"

using namespace std;
using namespace cv;
using namespace sen;

GLuint texture_id;
int texture_width;
int texture_height;

int bufferIndex = 0;
std::thread capturing_thread;
std::mutex mtx;
int glutwin;
unsigned char key;

k4a_device_configuration_t config;
k4a::device device;
k4a::capture capture;

k4a::image initTexture;
k4a::image colorImageBuffer;
std::array<k4a::image, 30> colorBuffer;
k4a::image outColorFrame;

k4a::calibration calibration;
Mat camera_matrix_color;
Mat dist_coeffs_color;

aruco::Dictionary dic;
Mat generatedMarker;
aruco::CameraParameters CamParam;
aruco::MarkerDetector MDetector;
std::map<int, aruco::MarkerPoseTracker> MTracker;
float MarkerSize = 0.1f;
double proj_matrix[16];
double modelview_matrix[16];

void drawCubeModel()
{
	static const GLfloat LightAmbient[] = { 0.25f, 0.25f, 0.25f, 1.0f };
	static const GLfloat LightDiffuse[] = { 0.1f, 0.1f, 0.1f, 1.0f };   
	static const GLfloat LightPosition[] = { 0.0f, 0.0f, 2.0f, 1.0f };  

	glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_POLYGON_BIT);

	glColor4f(0.2f, 0.35f, 0.3f, 0.75f);        
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	glShadeModel(GL_SMOOTH);

	glEnable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
	glEnable(GL_COLOR_MATERIAL);

	glScalef(0.05, 0.05, 0.05);
	glTranslatef(0, 0, 1);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_QUADS);

	glNormal3f(0.0f, 0.0f, 1.0f);  
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f); 
	glVertex3f(1.0f, 1.0f, 1.0f);  
	glVertex3f(-1.0f, 1.0f, 1.0f); 

	glNormal3f(0.0f, 0.0f, -1.0f);  
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f); 
	glVertex3f(1.0f, 1.0f, -1.0f);  
	glVertex3f(1.0f, -1.0f, -1.0f); 

	glNormal3f(0.0f, 1.0f, 0.0f);   
	glVertex3f(-1.0f, 1.0f, -1.0f); 
	glVertex3f(-1.0f, 1.0f, 1.0f);  
	glVertex3f(1.0f, 1.0f, 1.0f);  
	glVertex3f(1.0f, 1.0f, -1.0f);  

	glNormal3f(0.0f, -1.0f, 0.0f);  
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f); 
	glVertex3f(1.0f, -1.0f, 1.0f);  
	glVertex3f(-1.0f, -1.0f, 1.0f); 

	glNormal3f(1.0f, 0.0f, 0.0f);   
	glVertex3f(1.0f, -1.0f, -1.0f); 
	glVertex3f(1.0f, 1.0f, -1.0f);  
	glVertex3f(1.0f, 1.0f, 1.0f);  
	glVertex3f(1.0f, -1.0f, 1.0f);  

	glNormal3f(-1.0f, 0.0f, 0.0f);  
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f); 
	glVertex3f(-1.0f, 1.0f, 1.0f);  
	glVertex3f(-1.0f, 1.0f, -1.0f); 
	glEnd();

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glColor4f(0.2f, 0.65f, 0.3f, 0.35f);
	glBegin(GL_QUADS);

	glNormal3f(0.0f, 0.0f, 1.0f);   
	glVertex3f(-1.0f, -1.0f, 1.0f); 
	glVertex3f(1.0f, -1.0f, 1.0f);  
	glVertex3f(1.0f, 1.0f, 1.0f);  
	glVertex3f(-1.0f, 1.0f, 1.0f);  

	glNormal3f(0.0f, 0.0f, -1.0f);  
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f); 
	glVertex3f(1.0f, 1.0f, -1.0f);  
	glVertex3f(1.0f, -1.0f, -1.0f); 

	glNormal3f(0.0f, 1.0f, 0.0f);   
	glVertex3f(-1.0f, 1.0f, -1.0f); 
	glVertex3f(-1.0f, 1.0f, 1.0f);  
	glVertex3f(1.0f, 1.0f, 1.0f);  
	glVertex3f(1.0f, 1.0f, -1.0f);  

	glNormal3f(0.0f, -1.0f, 0.0f);  
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f); 
	glVertex3f(1.0f, -1.0f, 1.0f);  
	glVertex3f(-1.0f, -1.0f, 1.0f); 

	glNormal3f(1.0f, 0.0f, 0.0f);   
	glVertex3f(1.0f, -1.0f, -1.0f); 
	glVertex3f(1.0f, 1.0f, -1.0f); 
	glVertex3f(1.0f, 1.0f, 1.0f);  
	glVertex3f(1.0f, -1.0f, 1.0f);  

	glNormal3f(-1.0f, 0.0f, 0.0f);  
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f); 
	glVertex3f(-1.0f, 1.0f, 1.0f);  
	glVertex3f(-1.0f, 1.0f, -1.0f); 
	glEnd();

	glPopAttrib();
}
void axis(float size)
{
	glColor3f(1, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(0.0f, 0.0f, 0.0f);  
	glVertex3f(size, 0.0f, 0.0f);  
	glEnd();

	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex3f(0.0f, 0.0f, 0.0f);  
	glVertex3f(0.0f, size, 0.0f);  
	glEnd();

	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(0.0f, 0.0f, 0.0f);  
	glVertex3f(0.0f, 0.0f, size);  
	glEnd();
}
void drawAugmentedScene()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glLoadMatrixd(proj_matrix);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glm::mat4 Model = glm::make_mat4(modelview_matrix);
	glLoadMatrixf(glm::value_ptr(Model));
	axis(0.1);
	drawCubeModel();
	
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
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

		if (key == 'q')
		{
			glutLeaveMainLoop();
			break;
		}

		mtx.lock();
		colorBuffer[(bufferIndex++) % 30] = colorImageBuffer;
		mtx.unlock();

		glutPostWindowRedisplay(glutwin);
	}
}

void createTexture(void)
{
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glGenTextures(1, &texture_id);
	glBindTexture(GL_TEXTURE_2D, texture_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

void drawBackground()
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture_id);

	if (bufferIndex > 0)
	{
		mtx.lock();

		outColorFrame = colorBuffer[(bufferIndex - 1) % 30];

		mtx.unlock();

		if (texture_id != 0)
		{
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture_width, texture_height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, outColorFrame.get_buffer());
		}
	}

	int w = texture_width;
	int h = texture_height;

	const GLfloat bgTextureVertices[] = { 0, 0, (float)w, 0, 0, (float)h, (float)w, (float)h };
	const GLfloat bgTextureCoords[] = { 1, 0, 1, 1, 0, 0, 0, 1 };
	const GLfloat proj[] = { 0, -2.f / w, 0, 0, -2.f / h, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1 };

	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(proj);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);

	glVertexPointer(2, GL_FLOAT, 0, bgTextureVertices);
	glTexCoordPointer(2, GL_FLOAT, 0, bgTextureCoords);

	glColor4f(1, 1, 1, 1);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);

	glDisable(GL_TEXTURE_2D);
}

void display()
{
	drawBackground();
	drawAugmentedScene();
	glutSwapBuffers();
}

void reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	CamParam.glGetProjectionMatrix(cv::Size(texture_width, texture_height), cv::Size(w, h), proj_matrix, 0.05, 10);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void keyboard(unsigned char button, int x, int y)
{
	key = button;
	glutPostRedisplay();
}
void idle()
{
	glutPostRedisplay();
}

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
	device = k4a::device::open(K4A_DEVICE_DEFAULT);
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
	
	texture_width = initTexture.get_width_pixels();
	texture_height = initTexture.get_height_pixels();

	calibration = device.get_calibration(config.depth_mode, config.color_resolution);
	k4a_calibration_intrinsic_parameters_t *intrinsics_color = &calibration.color_camera_calibration.intrinsics.parameters;
	vector<float> _camera_matrix = {
	   intrinsics_color->param.fx, 0.f, intrinsics_color->param.cx,
	   0.f, intrinsics_color->param.fy, intrinsics_color->param.cy,
	   0.f, 0.f, 1.f };
	camera_matrix_color = Mat(3, 3, CV_32F, &_camera_matrix[0]);
	vector<float> _dist_coeffs = { intrinsics_color->param.k1, intrinsics_color->param.k2, intrinsics_color->param.p1,
								   intrinsics_color->param.p2, intrinsics_color->param.k3, intrinsics_color->param.k4,
								   intrinsics_color->param.k5, intrinsics_color->param.k6 };
	dist_coeffs_color = Mat::zeros(5, 1, CV_32F);

	{
		dic = aruco::Dictionary::load("ARUCO_MIP_36h12");
		CamParam.setParams(camera_matrix_color, dist_coeffs_color, cv::Size(texture_width, texture_height));
		MDetector.setDictionary("ARUCO_MIP_36h12", 0.05f);
		MDetector.setDetectionMode(aruco::DM_VIDEO_FAST);
	}

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

	glutInitWindowSize(texture_width, texture_height);
	glutInitWindowPosition(100, 100);
	int glutwin = glutCreateWindow("test");

	createTexture();

	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);

	capturing_thread = std::thread(frameRetriever);
	capturing_thread.detach();

	glutMainLoop();
	capturing_thread.join();
	device.close();

	return 0;
}