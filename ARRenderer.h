#pragma once
#include "Shader.h"

namespace sen
{
	class ARRenderer
	{
	public:
		ARRenderer()
		{

		}

		bool setup(const std::string &vertex_shader_file, const std::string &fragment_shader_file);
		void render(double proj_matrix[16], double modelview_matrix[16]);
		void deleteBuffer();

	private:
		GLuint programID;
		GLuint MatrixID;
		GLuint TextureID;
		GLuint texture_id;
		GLuint vertexBuffer;
		GLuint colorBuffer;
		glm::mat4 MVP;
	};
}