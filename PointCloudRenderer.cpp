#include <iostream>

#include <Eigen/Dense>

#include <windows.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Texture.h"
#include "PointCloudRenderer.h"

using namespace std;
using namespace Eigen;
using namespace glm;


glm::vec3 position = glm::vec3(0, 0, 0);
float horizontalAngle = 0.0f;
float verticalAngle = 0.0f;

// Initial
float fovy = 60.0f;
float aspect = 1.0f;
float gNear = 0.01f;
float gFar = 1000.0f;
Eigen::Vector3f pos_vec;
Eigen::Vector3f focal_vec;
Eigen::Vector3f up_vec;
glm::mat4 MVP_original;

float speed = 1.0f;
float mouseSpeed = 0.002f;

void controlPointClouds(GLFWwindow* currentWindow, const int &window_width, const int &window_height, glm::mat4 &projectionMatrix, glm::mat4 &viewMatrix)
{
	static double lastTime = glfwGetTime();
	double currentTime = glfwGetTime();
	float deltaTime = float(currentTime - lastTime);

	double xpos, ypos;
	glfwGetCursorPos(currentWindow, &xpos, &ypos);
	glfwSetCursorPos(currentWindow, window_width / 2, window_height / 2);

	horizontalAngle += mouseSpeed * float(window_width / 2 - xpos);
	verticalAngle += mouseSpeed * float(window_height / 2 - ypos);

	glm::vec3 direction(
		cos(verticalAngle) * sin(horizontalAngle),
		sin(verticalAngle),
		cos(verticalAngle) * cos(horizontalAngle)
	);

	glm::vec3 right = glm::vec3(
		sin(horizontalAngle - 3.14f / 2.0f),
		0,
		cos(horizontalAngle - 3.14f / 2.0f)
	);

	glm::vec3 up = glm::cross(right, direction);

	if (glfwGetKey(currentWindow, GLFW_KEY_UP) == GLFW_PRESS)
	{
		position += direction * deltaTime * speed;
	}

	if (glfwGetKey(currentWindow, GLFW_KEY_DOWN) == GLFW_PRESS)
	{
		position -= direction * deltaTime * speed;
	}

	if (glfwGetKey(currentWindow, GLFW_KEY_RIGHT) == GLFW_PRESS)
	{
		position += right * deltaTime * speed;
	}

	if (glfwGetKey(currentWindow, GLFW_KEY_LEFT) == GLFW_PRESS)
	{
		position -= right * deltaTime * speed;
	}

	projectionMatrix = glm::perspective(glm::radians(fovy), aspect, gNear, gFar);
	viewMatrix = glm::lookAt(position,position + direction,up);

	lastTime = currentTime;
}

void sen::PointCloudRenderer::initializeMVP(const k4a::calibration &calibration)
{
	Eigen::Matrix3f intrinsics;
	Eigen::Matrix4f extrinsics;

	k4a_calibration_intrinsic_parameters_t intrinsics_color = calibration.color_camera_calibration.intrinsics.parameters;

	intrinsics <<
		intrinsics_color.param.fx, 0.f, intrinsics_color.param.cx,
		0.f, intrinsics_color.param.fy, intrinsics_color.param.cy,
		0.f, 0.f, 1.f;

	extrinsics = Eigen::Matrix4f::Identity();

	pos_vec = extrinsics.block<3, 1>(0, 3);
	Eigen::Matrix3f rotation = extrinsics.block<3, 3>(0, 0);
	Eigen::Vector3f y_axis(0.f, 1.f, 0.f);
	up_vec = Eigen::Vector3f(rotation * y_axis);

	Eigen::Vector3f z_axis(0.f, 0.f, 1.f);
	focal_vec = pos_vec + rotation * z_axis;

	Eigen::Vector2i window_size;
	window_size[0] = 2 * static_cast<int> (intrinsics(0, 2));
	window_size[1] = 2 * static_cast<int> (intrinsics(1, 2));

	fovy = 2 * atan(window_size[1] / (2. * intrinsics(1, 1))) * 180.0 / M_PI;

	aspect = (float)window_size[0] / (float)window_size[1];
	position = glm::make_vec3(pos_vec.data());

	ProjectionMatrix = glm::perspective(glm::radians(fovy), aspect, gNear, gFar);

	ViewMatrix = glm::lookAt(
		glm::vec3(pos_vec[0], pos_vec[1], pos_vec[2]),
		glm::vec3(focal_vec[0], focal_vec[1], focal_vec[2]),
		glm::vec3(up_vec[0], up_vec[1], up_vec[2]));

	glm::mat4 Model = mat4(1.0f);
	MVP = ProjectionMatrix * ViewMatrix * Model;
	MVP_original = MVP;
}

bool sen::PointCloudRenderer::setup(const std::string &vertex_shader_file, const std::string &fragment_shader_file, // => shader file path
	const int &texture_width, const int &texture_height, // => buffer data size
	const k4a::calibration &calibration// => k4a::transformation => convert depthImage & colorImage into point cloud
	)
{
	programID = LoadShaders(vertex_shader_file, fragment_shader_file);
	MatrixID = glGetUniformLocation(programID, "MVP");

	initializeMVP(calibration);

	transformation = k4a::transformation(calibration);
	int dataSize = texture_width * texture_height;
	glGenBuffers(1, &vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, dataSize * sizeof(glm::vec3), 0, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &colorBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
	glBufferData(GL_ARRAY_BUFFER, dataSize * sizeof(glm::vec3), 0, GL_DYNAMIC_DRAW);

	return true;
}

void sen::PointCloudRenderer::convertRGBDepthToPointXYZRGB(const k4a::image &colorImage, const k4a::image &depthImage, vector<glm::vec3> &vertices, vector<glm::vec3> &colors)
{
	int color_image_width_pixels = colorImage.get_width_pixels();
	int color_image_height_pixels = colorImage.get_height_pixels();

	k4a::image transformed_depth_image = NULL;
	transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * (int)sizeof(uint16_t));

	k4a::image point_cloud_image = NULL;
	point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * 3 * (int)sizeof(int16_t));

	transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
	transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

	int width = colorImage.get_width_pixels();
	int height = colorImage.get_height_pixels();

	int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();
	const uint8_t *color_image_data = colorImage.get_buffer();

	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitZ());

	vertices.resize(width * height);
	colors.resize(width * height);

	for (int i = 0; i < width * height; ++i)
	{
		Vector3f xyz = Vector3f(point_cloud_image_data[3 * i + 0] / 1000.0f,
			point_cloud_image_data[3 * i + 1] / 1000.0f,
			point_cloud_image_data[3 * i + 2] / 1000.0f);

		if (xyz(2) == 0.0f)
		{
			continue;
		}

		xyz = m * xyz;

		Vector3f rgb = Vector3f(color_image_data[4 * i + 2] / 256.0f,
			color_image_data[4 * i + 1] / 256.0f,
			color_image_data[4 * i + 0] / 256.0f);

		vertices[i] = glm::make_vec3(xyz.data());
		colors[i] = glm::make_vec3(rgb.data());
	}
}
void sen::PointCloudRenderer::render(GLFWwindow* currentWindow, const k4a::image &colorImage, const k4a::image &depthImage)
{
	int color_image_width_pixels = colorImage.get_width_pixels();
	int color_image_height_pixels = colorImage.get_height_pixels();

	vector<glm::vec3> vertices;
	vector<glm::vec3> colors;

	convertRGBDepthToPointXYZRGB(colorImage, depthImage, vertices, colors);

	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
	glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(glm::vec3), &colors[0], GL_DYNAMIC_DRAW);

	glUseProgram(programID);
	if (glfwGetMouseButton(currentWindow, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
	{
		controlPointClouds(currentWindow, color_image_width_pixels, color_image_height_pixels, ProjectionMatrix, ViewMatrix);
		glm::mat4 Model = mat4(1.0f);
		MVP = ProjectionMatrix * ViewMatrix * Model;
	}
	else
	{
		MVP = MVP_original;
	}
	
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

	glEnable(GL_PROGRAM_POINT_SIZE);
	glDrawArrays(GL_POINTS, 0, static_cast<int>(color_image_width_pixels * color_image_height_pixels) * 3);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
}
void sen::PointCloudRenderer::deleteBuffer()
{
	glDeleteBuffers(1, &vertexBuffer);
	glDeleteBuffers(1, &colorBuffer);
	glDeleteProgram(programID);
}