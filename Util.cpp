#include <iostream>

#ifdef USE_GL_3
#include <windows.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#endif

#include "Util.h"

using namespace std;

void sen::create_xy_table(const k4a::calibration &calibration, k4a::image &xy_table)
{
	k4a_float2_t *table_data = (k4a_float2_t *)(void *)xy_table.get_buffer();

	int width = calibration.depth_camera_calibration.resolution_width;
	int height = calibration.depth_camera_calibration.resolution_height;

	k4a_float2_t p;
	k4a_float3_t ray;
	int valid = 1;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;

			calibration.convert_2d_to_3d(p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray);

			if (valid)
			{
				table_data[idx].xy.x = ray.xyz.x;
				table_data[idx].xy.y = ray.xyz.y;
			}
			else
			{
				table_data[idx].xy.x = nanf("");
				table_data[idx].xy.y = nanf("");
			}
		}
	}
}
double sen::ElapsedTime(const LARGE_INTEGER& start, const LARGE_INTEGER& end)
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	return (double(end.QuadPart) - double(start.QuadPart)) * 1000.0 / double(freq.QuadPart);
}

#ifdef USE_GL_3
void sen::checkGlError(const char * op)
{
	for (GLint error = glGetError(); error; error = glGetError())
	{
		std::cout << "After " << op << "() glError (0x" << error << ")" << std::endl;
	}
}
#endif