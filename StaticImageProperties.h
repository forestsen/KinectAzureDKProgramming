#pragma once
#include <utility>
#include <k4a/k4a.hpp>

#include "Pixel.h"

namespace sen
{
	inline void ColorConvertHSVtoRGB(float h, float s, float v, float& out_r, float& out_g, float& out_b)
	{
		if (s == 0.0f)
		{
			// gray
			out_r = out_g = out_b = v;
			return;
		}

		h = fmodf(h, 1.0f) / (60.0f / 360.0f);
		int   i = (int)h;
		float f = h - (float)i;
		float p = v * (1.0f - s);
		float q = v * (1.0f - s * f);
		float t = v * (1.0f - s * (1.0f - f));

		switch (i)
		{
		case 0: out_r = v; out_g = t; out_b = p; break;
		case 1: out_r = q; out_g = v; out_b = p; break;
		case 2: out_r = p; out_g = v; out_b = t; break;
		case 3: out_r = p; out_g = q; out_b = v; break;
		case 4: out_r = t; out_g = p; out_b = v; break;
		case 5: default: out_r = v; out_g = p; out_b = q; break;
		}
	}

	inline std::pair<uint16_t, uint16_t> GetDepthModeRange(const k4a_depth_mode_t depthMode)
	{
		switch (depthMode)
		{
		case K4A_DEPTH_MODE_NFOV_2X2BINNED:
			return { (uint16_t)500, (uint16_t)5800 };
		case K4A_DEPTH_MODE_NFOV_UNBINNED:
			return { (uint16_t)500, (uint16_t)4000 };
		case K4A_DEPTH_MODE_WFOV_2X2BINNED:
			return { (uint16_t)250, (uint16_t)3000 };
		case K4A_DEPTH_MODE_WFOV_UNBINNED:
			return { (uint16_t)250, (uint16_t)2500 };

		case K4A_DEPTH_MODE_PASSIVE_IR:
		default:
			throw std::logic_error("Invalid depth mode!");
		}
	}

	inline std::pair<int, int> GetDepthDimensions(const k4a_depth_mode_t depthMode)
	{
		switch (depthMode)
		{
		case K4A_DEPTH_MODE_NFOV_2X2BINNED:
			return { 320, 288 };
		case K4A_DEPTH_MODE_NFOV_UNBINNED:
			return { 640, 576 };
		case K4A_DEPTH_MODE_WFOV_2X2BINNED:
			return { 512, 512 };
		case K4A_DEPTH_MODE_WFOV_UNBINNED:
			return { 1024, 1024 };
		case K4A_DEPTH_MODE_PASSIVE_IR:
			return { 1024, 1024 };

		default:
			throw std::logic_error("Invalid depth dimensions value!");
		}
	}

	inline std::pair<int, int> GetColorDimensions(const k4a_color_resolution_t resolution)
	{
		switch (resolution)
		{
		case K4A_COLOR_RESOLUTION_720P:
			return { 1280, 720 };
		case K4A_COLOR_RESOLUTION_2160P:
			return { 3840, 2160 };
		case K4A_COLOR_RESOLUTION_1440P:
			return { 2560, 1440 };
		case K4A_COLOR_RESOLUTION_1080P:
			return { 1920, 1080 };
		case K4A_COLOR_RESOLUTION_3072P:
			return { 4096, 3072 };
		case K4A_COLOR_RESOLUTION_1536P:
			return { 2048, 1536 };

		default:
			throw std::logic_error("Invalid color dimensions value!");
		}
	}

	inline std::pair<uint16_t, uint16_t> GetIrLevels(const k4a_depth_mode_t depthMode)
	{
		switch (depthMode)
		{
		case K4A_DEPTH_MODE_PASSIVE_IR:
			return { (uint16_t)0, (uint16_t)100 };

		case K4A_DEPTH_MODE_OFF:
			throw std::logic_error("Invalid depth mode!");

		default:
			return { (uint16_t)0, (uint16_t)1000 };
		}
	}

	using DepthPixelVisualizationFunction = Pixel(const DepthPixel &value, const DepthPixel &min, const DepthPixel &max);

	void ColorizeDepthImage(const k4a::image &depthImage,
		DepthPixelVisualizationFunction visualizationFn,
		std::pair<uint16_t, uint16_t> expectedValueRange,
		std::vector<Pixel> *buffer);
}