/* ImageColorAveraging.h
*
*  MIT License
*
*  Copyright (c) 2020-2026 awawa-dev
*
*  Project homesite: https://github.com/awawa-dev/HyperHDR
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  The above copyright notice and this permission notice shall be included in all
*  copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*  SOFTWARE.
*/

#pragma once

#ifndef PCH_ENABLED
	#include <cstdint>
	#include <vector>
	#include <map>
#endif

#include <base/LedString.h>
#include <image/Image.h>
#include <image/ColorRgb.h>
#include <utils/Logger.h>
#include <linalg.h>

namespace hyperhdr
{
	/**
	 * Tunable parameters for the LED zone color extraction algorithms.
	 * Used by the "dominant", "oklab", "kmeans" and "multisample" mapping types.
	 * All values are read from schema-color.json and can be changed at runtime
	 * from the WebUI.
	 */
	struct MappingParams
	{
		int   hueBins          = 18;    // number of hue buckets (4..64)
		int   neighborRadius   = 1;     // dominant cluster = peak +- N buckets (0..4)
		float minSaturation    = 0.15f; // below this saturation the pixel is "neutral"
		float valueGamma       = 0.80f; // gamma applied to the brightness weight (0..2)
		float neutralWeight    = 0.05f; // how much neutral pixels contribute (0..1)
		float saturationWeight = 2.0f;  // multiplier on saturation weight (0..10)
	};

	class ImageColorAveraging
	{
	public:
		ImageColorAveraging(
			const LoggerName& _log,
			const int mappingType,
			const bool sparseProcessing,
			const unsigned width,
			const unsigned height,
			const unsigned horizontalBorder,
			const unsigned verticalBorder,
			const quint8 instanceIndex,
			const std::vector<LedString::Led>& leds,
			const MappingParams& mappingParams);

		unsigned width() const;
		unsigned height() const;
		unsigned horizontalBorder() const;
		unsigned verticalBorder() const;

		// Processes the image and fills the ledColors vector with the resulting
		// per-LED colors. Color values are in linear RGB, normalized to 0..1.
		void process(std::vector<linalg::aliases::float3>& ledColors, const Image<ColorRgb>& image);

	private:
		// Classic modes
		void   getUnicolorForLeds(std::vector<linalg::aliases::float3>& ledColors, const Image<ColorRgb>& image) const;
		linalg::aliases::float3 calcUnicolorForLeds(const Image<ColorRgb>& image) const;

		void   getMulticolorForLeds(std::vector<linalg::aliases::float3>& ledColors, const Image<ColorRgb>& image) const;
		linalg::aliases::float3 calcMulticolorForLeds(const Image<ColorRgb>& image, const std::vector<uint32_t>& colors) const;

		// Dominant color extraction (Philips Hue / LG Ambilight style)
		void   getDominantForLeds(std::vector<linalg::aliases::float3>& ledColors, const Image<ColorRgb>& image) const;
		linalg::aliases::float3 calcDominantForLeds(const Image<ColorRgb>& image, const std::vector<uint32_t>& colors) const;

		// Oklab average (perceptually uniform averaging)
		void   getOklabForLeds(std::vector<linalg::aliases::float3>& ledColors, const Image<ColorRgb>& image) const;
		linalg::aliases::float3 calcOklabForLeds(const Image<ColorRgb>& image, const std::vector<uint32_t>& colors) const;

		// K-means (k=2): clusters the zone into 2 colors and returns the bigger one
		void   getKMeansForLeds(std::vector<linalg::aliases::float3>& ledColors, const Image<ColorRgb>& image) const;
		linalg::aliases::float3 calcKMeansForLeds(const Image<ColorRgb>& image, const std::vector<uint32_t>& colors) const;

		// Multi-sample voting: splits the zone into 4 sub-samples and votes
		void   getMultiSampleForLeds(std::vector<linalg::aliases::float3>& ledColors, const Image<ColorRgb>& image) const;
		linalg::aliases::float3 calcMultiSampleForLeds(const Image<ColorRgb>& image, const std::vector<uint32_t>& colors) const;

		LoggerName _log;
		unsigned _width;
		unsigned _height;
		bool     _sparseProcessing;
		unsigned _horizontalBorder;
		unsigned _verticalBorder;

		std::vector<std::vector<uint32_t>>           _colorsMap;
		std::map<int, std::vector<int>>              _colorGroups;
		int                                          _mappingType;
		MappingParams                                _mappingParams;
	};
}