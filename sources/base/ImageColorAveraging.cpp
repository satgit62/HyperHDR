/* ImageColorAveraging.cpp
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

#include <base/ImageColorAveraging.h>
#include <base/ImageToLedManager.h>
#include <infinite-color-engine/InfiniteProcessing.h>
#include <infinite-color-engine/ColorSpace.h>

#include <algorithm>
#include <array>
#include <limits>
#include <cstring>
#include <ranges>
#include <iterator>

#define push_back_index(list, index) list.push_back((index) * 3)

using namespace hyperhdr;
using namespace linalg::aliases;

namespace
{
	// Fast cube root approximation, ~10x faster than std::cbrt.
	// Bit-twiddling initial guess + one Newton-Raphson refinement.
	// Plenty accurate for color math (relative error < 0.5%).
	inline float fast_cbrt(float x)
	{
		if (x <= 0.0f) return 0.0f;
		union { int i; float f; } v;
		v.f = x;
		v.i = 0x2a5120a8 + v.i / 3;
		float y = v.f;
		y = y - (y - x / (y * y)) / 3.0f;
		return y;
	}

	// Upper bound for the hue bin count (see MappingParams.hueBins).
	constexpr int MAX_HUE_BINS = 64;
}

// ===========================================================================
//  Constructor / basic accessors
// ===========================================================================

ImageColorAveraging::ImageColorAveraging(
				const LoggerName& _log,
				const int mappingType,
				const bool sparseProcessing,
				const unsigned width,
				const unsigned height,
				const unsigned horizontalBorder,
				const unsigned verticalBorder,
				const quint8 /*instanceIndex*/,
				const std::vector<LedString::Led>& leds,
				const MappingParams& mappingParams)
	: _width(width)
	, _height(height)
	, _sparseProcessing(sparseProcessing)
	, _horizontalBorder(horizontalBorder)
	, _verticalBorder(verticalBorder)
	, _colorsMap()
	, _colorGroups()
	, _mappingParams(mappingParams)
{
	Q_ASSERT(_width > 2 * _verticalBorder);
	Q_ASSERT(_height > 2 * _horizontalBorder);
	Q_ASSERT(_width < 10000);
	Q_ASSERT(_height < 10000);

	_mappingType = mappingType;

	_colorsMap.reserve(leds.size());

	const int32_t xOffset = _verticalBorder;
	const int32_t actualWidth = _width - 2 * _verticalBorder;
	const int32_t yOffset = _horizontalBorder;
	const int32_t actualHeight = _height - 2 * _horizontalBorder;

	size_t   totalCount = 0;
	size_t   totalCapasity = 0;

	for (int ledIndex = -1; const LedString::Led& led : leds)
	{
		ledIndex++;

		if ((led.maxX_frac - led.minX_frac) < 1e-6 || (led.maxY_frac - led.minY_frac) < 1e-6)
		{
			_colorsMap.emplace_back();
			continue;
		}

		int32_t minX_idx = xOffset + int32_t(qRound(actualWidth * led.minX_frac));
		int32_t maxX_idx = xOffset + int32_t(qRound(actualWidth * led.maxX_frac));
		int32_t minY_idx = yOffset + int32_t(qRound(actualHeight * led.minY_frac));
		int32_t maxY_idx = yOffset + int32_t(qRound(actualHeight * led.maxY_frac));

		minX_idx = qMin(minX_idx, xOffset + actualWidth - 1);
		if (minX_idx == maxX_idx)
		{
			maxX_idx++;
		}
		minY_idx = qMin(minY_idx, yOffset + actualHeight - 1);
		if (minY_idx == maxY_idx)
		{
			maxY_idx++;
		}

		const int32_t maxYLedCount = qMin(maxY_idx, yOffset + actualHeight);
		const int32_t maxXLedCount = qMin(maxX_idx, xOffset + actualWidth);
		const int32_t realYLedCount = qAbs(maxYLedCount - minY_idx);
		const int32_t realXLedCount = qAbs(maxXLedCount - minX_idx);

		bool   sparseIndexes = sparseProcessing;
		size_t totalSize = static_cast<size_t>(realYLedCount) * realXLedCount;

		if (!sparseIndexes && totalSize > 1600)
		{
			Warning(_log, "This is large image area for lamp: {:d}. It contains {:d} indexes for captured video frame so reduce it by four. Enabling 'sparse processing' option for you. Consider to enable it permanently in the processing configuration to hide that warning.", ledIndex, totalSize);
			sparseIndexes = true;
		}

		const int32_t increment = (sparseIndexes) ? 2 : 1;

		totalCount += _colorsMap.emplace_back([&]{
			std::vector<uint32_t> ledColor;
			if (!led.disabled)
			{
				ledColor.reserve((sparseIndexes) ? ((static_cast<unsigned long long>(realYLedCount / 2)) + (realYLedCount % 2)) * ((realXLedCount / 2) + (realXLedCount % 2)) : totalSize);

				for (int32_t y = minY_idx; y < maxYLedCount; y += increment)
				{
					for (int32_t x = minX_idx; x < maxXLedCount; x += increment)
					{
						push_back_index(ledColor, y * width + x);
					}
				}
			}
			return ledColor;
		}()).size();
		totalCapasity += _colorsMap.back().capacity();

		if (led.group > 0)
		{
			if (_colorGroups.contains(led.group))
			{
				int master = _colorGroups[led.group].front();
				auto& dest_vec = _colorsMap[master];
				auto& source_vec = _colorsMap.back();

				dest_vec.insert(
					dest_vec.end(),
					std::make_move_iterator(source_vec.begin()),
					std::make_move_iterator(source_vec.end())
				);

				source_vec.clear();
			}
			_colorGroups[led.group].push_back(ledIndex);
		}
	}
	Info(_log, "Total index number is: {:d} (memory: {:d}). User sparse processing is: {:s}, image size: {:d} x {:d}, area number: {:d}",
		totalCount, totalCapasity, (sparseProcessing) ? "enabled" : "disabled", width, height, leds.size());
}

unsigned ImageColorAveraging::width() const
{
	return _width;
}

unsigned ImageColorAveraging::height() const
{
	return _height;
}

unsigned ImageColorAveraging::horizontalBorder() const
{
	return _horizontalBorder;
}

unsigned ImageColorAveraging::verticalBorder() const {
	return _verticalBorder;
}

// ===========================================================================
//  Main dispatch
// ===========================================================================

void ImageColorAveraging::process(std::vector<float3>& ledColors, const Image<ColorRgb>& image)
{
	ledColors.clear();
	ledColors.reserve(_colorsMap.size());

	switch (_mappingType)
	{
		case 1: getUnicolorForLeds(ledColors, image);    break;
		case 2: getDominantForLeds(ledColors, image);    break;
		case 3: getOklabForLeds(ledColors, image);       break;
		case 4: getKMeansForLeds(ledColors, image);      break;
		case 5: getMultiSampleForLeds(ledColors, image); break;
		default: getMulticolorForLeds(ledColors, image);
	}

	if (!_colorGroups.empty() && _mappingType != 1)
	{
		for (auto& group : _colorGroups)
		{
			const auto& combined = ledColors[group.second.front()];
			for (int g = 1; g < static_cast<int>(group.second.size()); g++)
			{
				ledColors[group.second[g]] = combined;
			}
		}
	}
}

// ===========================================================================
//  Classic modes: unicolor and multicolor (unchanged)
// ===========================================================================

void ImageColorAveraging::getUnicolorForLeds(std::vector<float3>& ledColors, const Image<ColorRgb>& image) const
{
	ledColors.resize(_colorsMap.size(), calcUnicolorForLeds(image));
}

void ImageColorAveraging::getMulticolorForLeds(std::vector<float3>& ledColors, const Image<ColorRgb>& image) const
{
	for (auto colors = _colorsMap.begin(); colors != _colorsMap.end(); ++colors)
	{
		ledColors.push_back(calcMulticolorForLeds(image, *colors));
	}
}

float3 ImageColorAveraging::calcMulticolorForLeds(const Image<ColorRgb>& image, const std::vector<uint32_t>& colors) const
{
	if (colors.empty())
	{
		return float3{ 0, 0, 0 };
	}

	linalg::vec<uint_fast64_t, 3> sumLinear(0, 0, 0);

	const uint8_t* imgData = image.rawMem();

	for (const uint32_t colorOffset : colors)
	{
		sumLinear += InfiniteProcessing::srgbNonlinearToLinear(byte3(imgData[colorOffset], imgData[colorOffset + 1], imgData[colorOffset + 2]));
	}

	auto averageLinear = (static_cast<float3>(sumLinear) / static_cast<float>(colors.size())) / 65535.0f;

	return averageLinear;
}

float3 ImageColorAveraging::calcUnicolorForLeds(const Image<ColorRgb>& image) const
{
	uint_fast64_t sum = 0;
	linalg::vec<uint_fast64_t, 3> sumLinear(0, 0, 0);

	const uint8_t* imgData = image.rawMem();
	const uint32_t rowSize = image.width() * static_cast<uint32_t>(sizeof(ColorRgb));
	const uint32_t increment = (_sparseProcessing) ? 2 : 1;

	for (uint32_t y = 0; y < image.height(); y += increment)
	{
		for (uint32_t colorOffset = y * rowSize; colorOffset < y * rowSize + rowSize; colorOffset += increment * static_cast<uint32_t>(sizeof(ColorRgb)))
		{
			sumLinear += InfiniteProcessing::srgbNonlinearToLinear(byte3(imgData[colorOffset], imgData[colorOffset + 1], imgData[colorOffset + 2]));
			sum++;
		}
	}

	auto averageLinear = (static_cast<float3>(sumLinear) / static_cast<float>(sum)) / 65535.0f;

	return averageLinear;
}

// ===========================================================================
//  Dominant color extraction (Philips Hue / LG Ambilight style)
//
//  Optimizations vs. the naive version:
//    - Gamma LUT instead of std::pow() per pixel (biggest win)
//    - Inline HSV, no rgb2hsv() call and no fmod
//    - Stack-allocated bin arrays, no per-zone heap allocation
//    - Float accumulators, no uint16<->uint64 conversion
//    - Division-free saturation threshold check
// ===========================================================================

void ImageColorAveraging::getDominantForLeds(std::vector<float3>& ledColors, const Image<ColorRgb>& image) const
{
	ledColors.reserve(_colorsMap.size());
	for (const auto& colors : _colorsMap)
	{
		ledColors.push_back(calcDominantForLeds(image, colors));
	}
}

float3 ImageColorAveraging::calcDominantForLeds(const Image<ColorRgb>& image, const std::vector<uint32_t>& colors) const
{
	if (colors.empty())
	{
		return float3{ 0, 0, 0 };
	}

	// Pull the tunable parameters (see ImageColorAveraging.h)
	const int   HUE_BINS          = std::max(4, std::min(_mappingParams.hueBins, MAX_HUE_BINS));
	const int   NEIGHBOR_RADIUS   = std::max(0, std::min(_mappingParams.neighborRadius, 4));
	const float MIN_SATURATION    = std::clamp(_mappingParams.minSaturation, 0.0f, 1.0f);
	const float VALUE_GAMMA       = std::clamp(_mappingParams.valueGamma, 0.05f, 2.0f);
	const float NEUTRAL_WEIGHT    = std::clamp(_mappingParams.neutralWeight, 0.0f, 1.0f);
	const float SATURATION_WEIGHT = std::clamp(_mappingParams.saturationWeight, 0.0f, 10.0f);

	// --- Gamma LUT --------------------------------------------------------
	// Rebuilt only when VALUE_GAMMA changes. thread_local is safe because the
	// capture pipeline runs on a single thread.
	static thread_local float cachedGamma = -1.0f;
	static thread_local std::array<float, 256> gammaLut{};

	if (cachedGamma != VALUE_GAMMA)
	{
		for (int i = 0; i < 256; ++i)
		{
			const float v = static_cast<float>(i) / 255.0f;
			gammaLut[i] = std::pow(std::max(v, 0.02f), VALUE_GAMMA);
		}
		cachedGamma = VALUE_GAMMA;
	}

	// --- Stack-allocated accumulators ------------------------------------
	float  binWeight[MAX_HUE_BINS];
	float3 binSum[MAX_HUE_BINS];

	for (int i = 0; i < HUE_BINS; ++i)
	{
		binWeight[i] = 0.0f;
		binSum[i] = float3{ 0, 0, 0 };
	}

	float  neutralWeight = 0.0f;
	float3 neutralSum{ 0, 0, 0 };

	const float binScale = static_cast<float>(HUE_BINS) / 6.0f;
	const uint8_t* imgData = image.rawMem();

	for (const uint32_t off : colors)
	{
		const uint8_t r8 = imgData[off];
		const uint8_t g8 = imgData[off + 1];
		const uint8_t b8 = imgData[off + 2];

		// Cheap min/max without a full HSV conversion
		const uint8_t max8   = std::max({ r8, g8, b8 });
		const uint8_t min8   = std::min({ r8, g8, b8 });
		const uint8_t delta8 = max8 - min8;

		const float maxf   = max8 * (1.0f / 255.0f);
		const float deltaf = delta8 * (1.0f / 255.0f);
		const float sat    = (max8 > 0) ? (deltaf / maxf) : 0.0f;

		// Weight: saturated & bright pixels dominate.
		const float valFactor = gammaLut[max8];
		const float weight    = (1.0f + sat * SATURATION_WEIGHT) * valFactor;

		// Convert to linear RGB using the existing 256-entry LUT
		const linalg::vec<uint16_t, 3> linearU16 =
			InfiniteProcessing::srgbNonlinearToLinear(byte3(r8, g8, b8));

		const float3 linear{
			linearU16.x * (1.0f / 65535.0f),
			linearU16.y * (1.0f / 65535.0f),
			linearU16.z * (1.0f / 65535.0f)
		};

		if (delta8 == 0 || sat < MIN_SATURATION)
		{
			neutralWeight += weight;
			neutralSum.x += linear.x * weight;
			neutralSum.y += linear.y * weight;
			neutralSum.z += linear.z * weight;
			continue;
		}

		// Inline hue in [0, 6). No fmod needed.
		const float rf = r8;
		const float gf = g8;
		const float bf = b8;
		const float d  = static_cast<float>(delta8);

		float hue;
		if (max8 == r8)      hue = (gf - bf) / d;
		else if (max8 == g8) hue = 2.0f + (bf - rf) / d;
		else                 hue = 4.0f + (rf - gf) / d;

		if (hue < 0.0f) hue += 6.0f;

		int bin = static_cast<int>(hue * binScale);
		if (bin < 0) bin = 0;
		else if (bin >= HUE_BINS) bin = HUE_BINS - 1;

		binWeight[bin] += weight;
		binSum[bin].x  += linear.x * weight;
		binSum[bin].y  += linear.y * weight;
		binSum[bin].z  += linear.z * weight;
	}

	// --- Find the peak bin ------------------------------------------------
	int   peakBin    = 0;
	float peakWeight = 0.0f;
	for (int i = 0; i < HUE_BINS; ++i)
	{
		if (binWeight[i] > peakWeight)
		{
			peakWeight = binWeight[i];
			peakBin    = i;
		}
	}

	if (peakWeight <= 0.0f)
	{
		return calcMulticolorForLeds(image, colors);
	}

	// --- Sum the dominant cluster (peak +- neighbors) ---------------------
	float  clusterWeight = 0.0f;
	float3 clusterSum{ 0, 0, 0 };

	for (int d = -NEIGHBOR_RADIUS; d <= NEIGHBOR_RADIUS; ++d)
	{
		const int idx = (peakBin + d + HUE_BINS) % HUE_BINS;
		clusterWeight += binWeight[idx];
		clusterSum.x  += binSum[idx].x;
		clusterSum.y  += binSum[idx].y;
		clusterSum.z  += binSum[idx].z;
	}

	// Add a small neutral contribution so fully desaturated zones aren't crushed
	if (neutralWeight > 0.0f && NEUTRAL_WEIGHT > 0.0f)
	{
		clusterWeight += neutralWeight * NEUTRAL_WEIGHT;
		clusterSum.x  += neutralSum.x * NEUTRAL_WEIGHT;
		clusterSum.y  += neutralSum.y * NEUTRAL_WEIGHT;
		clusterSum.z  += neutralSum.z * NEUTRAL_WEIGHT;
	}

	if (clusterWeight <= 0.0f)
	{
		return calcMulticolorForLeds(image, colors);
	}

	const float inv = 1.0f / clusterWeight;

	return linalg::clamp(float3{
		clusterSum.x * inv,
		clusterSum.y * inv,
		clusterSum.z * inv
	}, 0.0f, 1.0f);
}

// ===========================================================================
//  Oklab average (perceptually uniform averaging)
//
//  Optimizations:
//    - fast_cbrt instead of std::cbrt (~10x faster)
//    - Unrolled 3x3 matrix products (no linalg::mul overhead)
//    - Integer-loop is linear per pixel, no branches
// ===========================================================================

void ImageColorAveraging::getOklabForLeds(std::vector<float3>& ledColors, const Image<ColorRgb>& image) const
{
	ledColors.reserve(_colorsMap.size());
	for (const auto& colors : _colorsMap)
	{
		ledColors.push_back(calcOklabForLeds(image, colors));
	}
}

float3 ImageColorAveraging::calcOklabForLeds(const Image<ColorRgb>& image, const std::vector<uint32_t>& colors) const
{
	if (colors.empty())
	{
		return float3{ 0, 0, 0 };
	}

	// Oklab M1 matrix (linear RGB -> LMS)
	constexpr float M1_00 = 0.4122214708f, M1_01 = 0.5363325363f, M1_02 = 0.0514459929f;
	constexpr float M1_10 = 0.2119034982f, M1_11 = 0.6806995451f, M1_12 = 0.1073969566f;
	constexpr float M1_20 = 0.0883024619f, M1_21 = 0.2817188376f, M1_22 = 0.6299787005f;

	// Oklab M2 matrix (LMS' -> Lab)
	constexpr float M2_00 =  0.2104542553f, M2_01 =  0.7936177850f, M2_02 = -0.0040720468f;
	constexpr float M2_10 =  1.9779984951f, M2_11 = -2.4285922050f, M2_12 =  0.4505937099f;
	constexpr float M2_20 =  0.0259040371f, M2_21 =  0.7827717662f, M2_22 = -0.8086757660f;

	float3 sumLab{ 0, 0, 0 };
	const uint8_t* imgData = image.rawMem();
	const float invPixels = 1.0f / static_cast<float>(colors.size());

	for (const uint32_t off : colors)
	{
		const auto linearU16 = InfiniteProcessing::srgbNonlinearToLinear(
			byte3(imgData[off], imgData[off + 1], imgData[off + 2]));

		const float r = linearU16.x * (1.0f / 65535.0f);
		const float g = linearU16.y * (1.0f / 65535.0f);
		const float b = linearU16.z * (1.0f / 65535.0f);

		// Linear RGB -> LMS, unrolled
		const float l = M1_00 * r + M1_01 * g + M1_02 * b;
		const float m = M1_10 * r + M1_11 * g + M1_12 * b;
		const float s = M1_20 * r + M1_21 * g + M1_22 * b;

		// Fast cube roots
		const float l_ = fast_cbrt(l);
		const float m_ = fast_cbrt(m);
		const float s_ = fast_cbrt(s);

		// LMS' -> Oklab, unrolled
		sumLab.x += M2_00 * l_ + M2_01 * m_ + M2_02 * s_;
		sumLab.y += M2_10 * l_ + M2_11 * m_ + M2_12 * s_;
		sumLab.z += M2_20 * l_ + M2_21 * m_ + M2_22 * s_;
	}

	// Average in Oklab
	const float3 avgLab{
		sumLab.x * invPixels,
		sumLab.y * invPixels,
		sumLab.z * invPixels
	};

	// Oklab -> linear RGB. Inverse matrix products, unrolled.
	constexpr float InvM2_00 =  1.0f,           InvM2_01 =  0.3963377774f, InvM2_02 =  0.2158037573f;
	constexpr float InvM2_10 =  1.0f,           InvM2_11 = -0.1055613458f, InvM2_12 = -0.0638541728f;
	constexpr float InvM2_20 =  1.0f,           InvM2_21 = -0.0894841775f, InvM2_22 = -1.2914855480f;

	constexpr float InvM1_00 =  4.0767416621f, InvM1_01 = -3.3077115913f, InvM1_02 =  0.2309699292f;
	constexpr float InvM1_10 = -1.2684380046f, InvM1_11 =  2.6097574011f, InvM1_12 = -0.3413193965f;
	constexpr float InvM1_20 = -0.0041960863f, InvM1_21 = -0.7034186147f, InvM1_22 =  1.7076147010f;

	const float lInv = InvM2_00 * avgLab.x + InvM2_01 * avgLab.y + InvM2_02 * avgLab.z;
	const float mInv = InvM2_10 * avgLab.x + InvM2_11 * avgLab.y + InvM2_12 * avgLab.z;
	const float sInv = InvM2_20 * avgLab.x + InvM2_21 * avgLab.y + InvM2_22 * avgLab.z;

	const float l3 = lInv * lInv * lInv;
	const float m3 = mInv * mInv * mInv;
	const float s3 = sInv * sInv * sInv;

	const float rOut = InvM1_00 * l3 + InvM1_01 * m3 + InvM1_02 * s3;
	const float gOut = InvM1_10 * l3 + InvM1_11 * m3 + InvM1_12 * s3;
	const float bOut = InvM1_20 * l3 + InvM1_21 * m3 + InvM1_22 * s3;

	return linalg::clamp(float3{ rOut, gOut, bOut }, 0.0f, 1.0f);
}

// ===========================================================================
//  K-means (k=2)
//
//  Optimizations:
//    - Precompute linear RGB once per pixel (was 4x)
//    - thread_local scratch buffer (no per-zone heap allocation)
//    - Float arithmetic (no uint16<->uint64 conversion)
//    - Early convergence exit (usually saves 1-2 iterations)
// ===========================================================================

void ImageColorAveraging::getKMeansForLeds(std::vector<float3>& ledColors, const Image<ColorRgb>& image) const
{
	ledColors.reserve(_colorsMap.size());
	for (const auto& colors : _colorsMap)
	{
		ledColors.push_back(calcKMeansForLeds(image, colors));
	}
}

float3 ImageColorAveraging::calcKMeansForLeds(const Image<ColorRgb>& image, const std::vector<uint32_t>& colors) const
{
	if (colors.empty())
	{
		return float3{ 0, 0, 0 };
	}

	const size_t n = colors.size();
	if (n < 4)
	{
		return calcMulticolorForLeds(image, colors);
	}

	const uint8_t* imgData = image.rawMem();

	// Reused scratch buffer. thread_local: capture pipeline runs on one thread,
	// but this protects us against future parallelization.
	static thread_local std::vector<float3> scratch;
	scratch.resize(n);

	// Precompute linear values once. This is the key optimization: the
	// k-means iterations below all operate on this flat array instead of
	// re-converting the image pixels on every iteration.
	float3 c0{ 0, 0, 0 };
	float3 c1{ 0, 0, 0 };

	{
		const uint32_t first = colors[0];
		const auto linFirst = InfiniteProcessing::srgbNonlinearToLinear(
			byte3(imgData[first], imgData[first + 1], imgData[first + 2]));
		c0 = float3{
			linFirst.x * (1.0f / 65535.0f),
			linFirst.y * (1.0f / 65535.0f),
			linFirst.z * (1.0f / 65535.0f)
		};
		scratch[0] = c0;

		float bestDist = -1.0f;
		for (size_t i = 1; i < n; ++i)
		{
			const uint32_t off = colors[i];
			const auto lin = InfiniteProcessing::srgbNonlinearToLinear(
				byte3(imgData[off], imgData[off + 1], imgData[off + 2]));

			const float3 p{
				lin.x * (1.0f / 65535.0f),
				lin.y * (1.0f / 65535.0f),
				lin.z * (1.0f / 65535.0f)
			};
			scratch[i] = p;

			const float dx = p.x - c0.x;
			const float dy = p.y - c0.y;
			const float dz = p.z - c0.z;
			const float dist = dx * dx + dy * dy + dz * dz;
			if (dist > bestDist)
			{
				bestDist = dist;
				c1 = p;
			}
		}
	}

	// Lloyd's algorithm, up to 4 iterations with early exit
	size_t n0 = 0;
	size_t n1 = 0;

	for (int iter = 0; iter < 4; ++iter)
	{
		float3 s0{ 0, 0, 0 };
		float3 s1{ 0, 0, 0 };
		n0 = 0;
		n1 = 0;

		for (size_t i = 0; i < n; ++i)
		{
			const float3& p = scratch[i];

			const float dx0 = p.x - c0.x;
			const float dy0 = p.y - c0.y;
			const float dz0 = p.z - c0.z;
			const float dx1 = p.x - c1.x;
			const float dy1 = p.y - c1.y;
			const float dz1 = p.z - c1.z;

			if (dx0 * dx0 + dy0 * dy0 + dz0 * dz0 <
			    dx1 * dx1 + dy1 * dy1 + dz1 * dz1)
			{
				s0.x += p.x; s0.y += p.y; s0.z += p.z;
				++n0;
			}
			else
			{
				s1.x += p.x; s1.y += p.y; s1.z += p.z;
				++n1;
			}
		}

		const float inv0 = (n0 > 0) ? 1.0f / static_cast<float>(n0) : 0.0f;
		const float inv1 = (n1 > 0) ? 1.0f / static_cast<float>(n1) : 0.0f;

		const float3 newC0 = (n0 > 0) ? float3{ s0.x * inv0, s0.y * inv0, s0.z * inv0 } : c0;
		const float3 newC1 = (n1 > 0) ? float3{ s1.x * inv1, s1.y * inv1, s1.z * inv1 } : c1;

		// Convergence check: if the centroids moved less than a tiny epsilon,
		// stop early. Saves ~1-2 iterations on average.
		const float dx0 = newC0.x - c0.x, dy0 = newC0.y - c0.y, dz0 = newC0.z - c0.z;
		const float dx1 = newC1.x - c1.x, dy1 = newC1.y - c1.y, dz1 = newC1.z - c1.z;
		const float move2 = dx0 * dx0 + dy0 * dy0 + dz0 * dz0
		                  + dx1 * dx1 + dy1 * dy1 + dz1 * dz1;

		c0 = newC0;
		c1 = newC1;

		if (move2 < 1e-6f)
		{
			break;
		}
	}

	// Return the bigger cluster (dominant population)
	const float3& bigger = (n0 >= n1) ? c0 : c1;
	return linalg::clamp(bigger, 0.0f, 1.0f);
}

// ===========================================================================
//  Multi-sample voting
//
//  Splits the zone pixels into 4 sub-samples and computes a dominant
//  color for each. The sample that is closest to all others wins.
//  Useful for very large zones (65" TV corners) where 4 sub-regions may
//  contain very different content.
//
//  With the optimized calcDominantForLeds (LUT gamma, inline HSV, stack
//  bins), the 4 calls are nearly free: the total pixel count is the same
//  as a single dominant call, only the small per-call overhead (bin init,
//  peak search, cluster sum) is multiplied by 4.
// ===========================================================================

void ImageColorAveraging::getMultiSampleForLeds(std::vector<float3>& ledColors, const Image<ColorRgb>& image) const
{
	ledColors.reserve(_colorsMap.size());
	for (const auto& colors : _colorsMap)
	{
		ledColors.push_back(calcMultiSampleForLeds(image, colors));
	}
}

float3 ImageColorAveraging::calcMultiSampleForLeds(const Image<ColorRgb>& image, const std::vector<uint32_t>& colors) const
{
	if (colors.empty())
	{
		return float3{ 0, 0, 0 };
	}

	const size_t n = colors.size();
	if (n < 16)
	{
		// Not enough pixels for a meaningful vote
		return calcDominantForLeds(image, colors);
	}

	const size_t quarter = n / 4;

	// Reuse a thread-local scratch buffer to avoid 4 heap allocations
	// per zone per frame. The capacity is retained across frames.
	static thread_local std::vector<uint32_t> scratch;

	float3 samples[4];
	for (int q = 0; q < 4; ++q)
	{
		const size_t begin = q * quarter;
		const size_t end   = (q == 3) ? n : (q + 1) * quarter;

		scratch.assign(colors.begin() + begin, colors.begin() + end);
		samples[q] = calcDominantForLeds(image, scratch);
	}

	// Vote: the sample closest to all others wins.
	// This is the "medoid" of the 4 samples in linear RGB space.
	int best = 0;
	float bestScore = std::numeric_limits<float>::max();

	for (int i = 0; i < 4; ++i)
	{
		float score = 0.0f;
		for (int j = 0; j < 4; ++j)
		{
			if (i == j) continue;
			const float dx = samples[i].x - samples[j].x;
			const float dy = samples[i].y - samples[j].y;
			const float dz = samples[i].z - samples[j].z;
			score += dx * dx + dy * dy + dz * dz;
		}
		if (score < bestScore)
		{
			bestScore = score;
			best = i;
		}
	}

	return samples[best];
}