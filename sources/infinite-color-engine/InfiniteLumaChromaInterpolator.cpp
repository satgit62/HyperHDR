/* InfiniteLumaChromaInterpolator.cpp
 *
 *  MIT License
 *  Copyright (c) 2020-2026 awawa-dev
 *
 *  Perceptually-inspired luma/chroma split smoothing in Oklab.
 */

#ifndef PCH_ENABLED
	#include <cmath>
	#include <algorithm>
	#include <vector>
	#include <iostream>
	#include <iomanip>
#endif

#include <infinite-color-engine/InfiniteLumaChromaInterpolator.h>

using namespace linalg::aliases;

namespace
{
	inline float srgbNlToLinear(float v)
	{
		v = std::clamp(v, 0.0f, 1.0f);
		return (v <= 0.04045f) ? (v / 12.92f)
		                       : std::pow((v + 0.055f) / 1.055f, 2.4f);
	}
	inline float srgbLinearToNl(float v)
	{
		v = std::clamp(v, 0.0f, 1.0f);
		return (v <= 0.0031308f) ? (v * 12.92f)
		                         : (1.055f * std::pow(v, 1.0f / 2.4f) - 0.055f);
	}
	inline float3 nlRgbToOklab(const float3& c)
	{
		const float r = srgbNlToLinear(c.x);
		const float g = srgbNlToLinear(c.y);
		const float b = srgbNlToLinear(c.z);

		const float l = 0.4122214708f * r + 0.5363325363f * g + 0.0514459929f * b;
		const float m = 0.2119034982f * r + 0.6806995451f * g + 0.1073969566f * b;
		const float s = 0.0883024619f * r + 0.2817188376f * g + 0.6299787005f * b;
		const float l_ = std::cbrt(l), m_ = std::cbrt(m), s_ = std::cbrt(s);

		return {
			 0.2104542553f * l_ + 0.7936177850f * m_ - 0.0040720468f * s_,
			 1.9779984951f * l_ - 2.4285922050f * m_ + 0.4505937099f * s_,
			 0.0259040371f * l_ + 0.7827717662f * m_ - 0.8086757660f * s_
		};
	}
	inline float3 oklabToNlRgb(const float3& lab)
	{
		const float l_ = lab.x + 0.3963377774f * lab.y + 0.2158037573f * lab.z;
		const float m_ = lab.x - 0.1055613458f * lab.y - 0.0638541728f * lab.z;
		const float s_ = lab.x - 0.0894841775f * lab.y - 1.2914855480f * lab.z;
		const float l = l_ * l_ * l_, m = m_ * m_ * m_, s = s_ * s_ * s_;

		const float r = 4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s;
		const float g = -1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s;
		const float b = -0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s;

		return { srgbLinearToNl(r), srgbLinearToNl(g), srgbLinearToNl(b) };
	}
}

InfiniteLumaChromaInterpolator::InfiniteLumaChromaInterpolator() = default;

void InfiniteLumaChromaInterpolator::setTransitionDuration(float durationMs)
{
	_lumaHalfLifeMs   = std::max(5.0f,  durationMs * 0.25f);
	_chromaHalfLifeMs = std::max(20.0f, durationMs * 0.90f);
}

void InfiniteLumaChromaInterpolator::setSmoothingFactor(float factor)
{
	factor = std::clamp(factor, 0.0f, 1.0f);
	_chromaHalfLifeMs = _lumaHalfLifeMs * (1.0f + factor * 3.0f);
}

void InfiniteLumaChromaInterpolator::resetState()
{
	_isAnimationComplete = true;
	_lastUpdate = 0;
	_L.clear(); _Lt.clear();
	_ab.clear(); _abt.clear();
}

void InfiniteLumaChromaInterpolator::resetToColors(std::vector<float3> colors, long long startTimeMs)
{
	if (colors.empty()) { _isAnimationComplete = true; return; }

	_L.resize(colors.size());  _Lt.resize(colors.size());
	_ab.resize(colors.size()); _abt.resize(colors.size());

	for (size_t i = 0; i < colors.size(); ++i)
	{
		const float3 lab = nlRgbToOklab(colors[i]);
		_L[i]  = lab.x;
		_ab[i] = float2{ lab.y, lab.z };
	}
	_Lt = _L;
	_abt = _ab;

	_isAnimationComplete = true;
	_lastUpdate = startTimeMs;
}

void InfiniteLumaChromaInterpolator::setTargetColors(std::vector<float3>&& new_rgb_targets,
                                                     long long startTimeMs, bool /*debug*/)
{
	if (new_rgb_targets.empty()) return;

	if (_L.size() != new_rgb_targets.size())
	{
		resetToColors(new_rgb_targets, startTimeMs);
		return;
	}

	for (size_t i = 0; i < new_rgb_targets.size(); ++i)
	{
		const float3 lab = nlRgbToOklab(new_rgb_targets[i]);
		_Lt[i]  = lab.x;
		_abt[i] = float2{ lab.y, lab.z };
	}
	_isAnimationComplete = false;
}

void InfiniteLumaChromaInterpolator::updateCurrentColors(long long currentTimeMs, float /*minBrightness*/)
{
	if (_isAnimationComplete || _L.empty()) { _lastUpdate = currentTimeMs; return; }

	const float dt = std::clamp(static_cast<float>(currentTimeMs - _lastUpdate), 0.001f, 100.0f);
	_lastUpdate = currentTimeMs;

	const float aL = 1.0f - std::pow(0.5f, dt / std::max(_lumaHalfLifeMs,   1.0f));
	const float aC = 1.0f - std::pow(0.5f, dt / std::max(_chromaHalfLifeMs, 1.0f));

	_isAnimationComplete = true;

	for (size_t i = 0; i < _L.size(); ++i)
	{
		_L[i]    += (_Lt[i]    - _L[i])    * aL;
		_ab[i].x += (_abt[i].x - _ab[i].x) * aC;
		_ab[i].y += (_abt[i].y - _ab[i].y) * aC;

		if (std::abs(_Lt[i] - _L[i]) > 0.002f ||
		    std::abs(_abt[i].x - _ab[i].x) > 0.002f ||
		    std::abs(_abt[i].y - _ab[i].y) > 0.002f)
			_isAnimationComplete = false;
	}
}

SharedOutputColors InfiniteLumaChromaInterpolator::getCurrentColors(float minBrightness)
{
	std::vector<float3> out(_L.size());
	for (size_t i = 0; i < _L.size(); ++i)
	{
		const float3 lab{ _L[i], _ab[i].x, _ab[i].y };
		out[i] = oklabToNlRgb(lab);
		if (minBrightness > 0.0f)
			out[i] = linalg::clamp(out[i], minBrightness, 1.0f);
	}
	return std::make_shared<std::vector<float3>>(std::move(out));
}

void InfiniteLumaChromaInterpolator::test()
{
	InfiniteLumaChromaInterpolator interp;
	interp.setTransitionDuration(150.0f);
	interp.setSmoothingFactor(0.5f);
	interp.resetToColors({ {0,0,0} }, 0);
	interp.setTargetColors({ {1,0,0} }, 0);

	std::cout << "LumaChroma test:\n";
	for (long long t = 0; t <= 500; t += 50)
	{
		interp.updateCurrentColors(t, 0.0f);
		auto c = interp.getCurrentColors();
		std::cout << std::setw(4) << t << "ms: "
		          << std::fixed << std::setprecision(3)
		          << (*c)[0].x << " " << (*c)[0].y << " " << (*c)[0].z << "\n";
	}
}