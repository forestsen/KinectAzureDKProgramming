#pragma once
#include <windows.h>

#include <string>

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

#ifdef CONVERT_IMAGE
#include "libyuv.h"
#include "turbojpeg.h"
#endif

namespace sen
{
	void create_xy_table(const k4a::calibration &calibration, k4a::image &xy_table);
	double ElapsedTime(const LARGE_INTEGER& start, const LARGE_INTEGER& end);
	void checkGlError(const char* op);
	inline uint32_t k4a_convert_fps_to_uint(k4a_fps_t fps)
	{
		uint32_t fps_int;
		switch (fps)
		{
		case K4A_FRAMES_PER_SECOND_5:
			fps_int = 5;
			break;
		case K4A_FRAMES_PER_SECOND_15:
			fps_int = 15;
			break;
		case K4A_FRAMES_PER_SECOND_30:
			fps_int = 30;
			break;
		default:
			fps_int = 0;
			break;
		}
		return fps_int;
	}
	inline bool ImagesAreCorrectlySized(const k4a::image &srcImage,
		const k4a::image &dstImage,
		const size_t *srcImageExpectedSize)
	{
		if (srcImageExpectedSize)
		{
			if (srcImage.get_size() != *srcImageExpectedSize)
			{
				return false;
			}
		}

		if (srcImage.get_width_pixels() != dstImage.get_width_pixels())
		{
			return false;
		}

		if (srcImage.get_height_pixels() != dstImage.get_height_pixels())
		{
			return false;
		}

		return true;
	}
#ifdef CONVERT_IMAGE
	bool ConvertImage(const k4a::image &srcImage, k4a::image *bgraImage);
#endif
}