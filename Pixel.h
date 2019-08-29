#pragma once
#include <cmath>

// Helper structs/typedefs to cast buffers to
//
namespace sen
{
	struct Pixel
	{
		uint8_t Blue;
		uint8_t Green;
		uint8_t Red;
		uint8_t Alpha;
	};

	using DepthPixel = uint16_t;
}