#pragma once
#include <string>

#include <gl/glew.h>
#include <GLFW/glfw3.h>

namespace sen
{
	GLuint LoadShaders(const std::string &vertex_file_path, const std::string &fragment_file_path);
}
