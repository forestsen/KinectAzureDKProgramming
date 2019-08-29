#pragma once
#include <vector>

#include <Eigen/Dense>

#include <k4a/k4a.hpp>

#include "Shader.h"

namespace sen
{
	class PointCloudRenderer
	{
	public:
		PointCloudRenderer()
		{

		}

		bool setup(const std::string &vertex_shader_file, const std::string &fragment_shader_file, 
			const int &texture_width, const int &texture_height,
			const k4a::calibration &calibration);
		void render(GLFWwindow* currentWindow, const k4a::image &colorFrame, const k4a::image &depthFrame);
		void deleteBuffer();

	private:
		void initializeMVP(const k4a::calibration &calibration);
		void convertRGBDepthToPointXYZRGB(const k4a::image &colorImage, const k4a::image &depthImage, std::vector<glm::vec3> &vertices, std::vector<glm::vec3> &colors);
		GLuint programID;
		GLuint MatrixID;
		GLuint TextureID;
		GLuint texture_id;
		GLuint vertexBuffer;
		GLuint colorBuffer;
		glm::mat4 ProjectionMatrix;
		glm::mat4 ViewMatrix;
		glm::mat4 MVP;
		k4a::transformation transformation;
	};
}