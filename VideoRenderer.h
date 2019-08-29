#pragma once
#include <k4a/k4a.hpp>

#include "Shader.h"

namespace sen
{
	class VideoRenderer
	{
	public:
		VideoRenderer()
		{

		}

		bool setup(const std::string &vertex_shader_file, const std::string &fragment_shader_file);
		void render(const k4a::image &frame);
		bool initTexture(const k4a::image &frame);
		void deleteBuffer();

	private:
		GLuint programID;
		GLuint MatrixID;
		GLuint TextureID;
		GLuint texture_id;
		GLuint vertexBuffer;
		GLuint uvBuffer;
		glm::mat4 MVP;
	};
}