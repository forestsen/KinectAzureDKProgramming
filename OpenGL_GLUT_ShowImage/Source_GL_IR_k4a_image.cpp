#include <iostream>
#include <vector>
#include <array>
#include <thread>
#include <mutex>

#include <windows.h>
#include <GL/freeglut.h>

#include <k4a/k4a.hpp>

#include "Pixel.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"

using namespace std;
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
k4a::image irImageBuffer;
std::array<k4a::image, 30> irBuffer;
k4a::image outIRFrame;

void frameRetriever()
{
	while (1)
	{
		if (device.get_capture(&capture, std::chrono::milliseconds(0)))
		{
			irImageBuffer = capture.get_ir_image();
		}

		if (key == 'q')
		{
			glutLeaveMainLoop();
			break;
		}

		mtx.lock();
		irBuffer[(bufferIndex++) % 30] = irImageBuffer;
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
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture_width, texture_height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, NULL);
	glBindTexture(GL_TEXTURE_2D, 0);
}
void drawBackground()
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture_id);

	if (bufferIndex > 0)
	{
		mtx.lock();

		outIRFrame = irBuffer[(bufferIndex - 1) % 30];

		mtx.unlock();

		if (texture_id != 0)
		{
			std::vector<sen::Pixel> irTextureBuffer;
			ColorizeDepthImage(outIRFrame, DepthPixelColorizer::ColorizeGreyscale, GetIrLevels(config.depth_mode), &irTextureBuffer);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture_width, texture_height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, irTextureBuffer.data());
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

	// Update attribute values.
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
	glutSwapBuffers();
}

void reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	createTexture();
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
	config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
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
			initTexture = capture.get_ir_image();
			break;
		}
	}

	texture_width = initTexture.get_width_pixels();
	texture_height = initTexture.get_height_pixels();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

	glutInitWindowSize(texture_width, texture_height);
	glutInitWindowPosition(100, 100);

	int glutwin = glutCreateWindow("test");

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