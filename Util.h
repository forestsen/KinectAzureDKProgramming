#pragma once
#include <windows.h>

#include <string>

#include <k4a/k4a.hpp>

namespace sen
{
	void create_xy_table(const k4a::calibration &calibration, k4a::image &xy_table);
	double ElapsedTime(const LARGE_INTEGER& start, const LARGE_INTEGER& end);
	void checkGlError(const char* op);
}