/* InfiniteKineticInterpolator.cpp
 *
 *  MIT License
 *  Copyright (c) 2020-2026 awawa-dev
 *
 *  Kinetic interpolator: Oklab + adaptive attack + scene detection
 *  + spatial smoothing + chroma boost.
 */

#ifndef PCH_ENABLED
	#include <cmath>
	#include <algorithm>
	#include <vector>
	#include <iostream>
	#include <iomanip>
#endif

#include <infinite-color-engine/InfiniteKineticInterpolator.h>

using namespace linalg::aliases;

namespace
{
	// ---- sRGB gamma -------------------------------------------------------
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

	inline float3 srgbNlToLinear3(const float3& c)
	{
		return { srgbNlToLinear(c.x), srgbNlToLinear(c.y), srgbNlToLinear(c.z) };
	}

	inline float3 srgbLinearToNl3(const float3& c)
	{
		return { srgbLinearToNl(c.x), srgbLinearToNl(c.y), srgbLinearToNl(c.z) };
	}

	// ---- Oklab (Björn Ottosson 2020) -------------------------------------
	inline float3 linearRgbToOklab(const float3& rgb)
	{
		const float l = 0.4122214708f * rgb.x + 0.5363325363f * rgb.y + 0.0514459929f * rgb.z;
		const float m = 0.2119034982f * rgb.x + 0.6806995451f * rgb.y + 0.1073969566f * rgb.z;
		const float s = 0.0883024619f * rgb.x + 0.2817188376f * rgb.y + 0.6299787005f * rgb.z;

		const float l_ = std::cbrt(l);
		const float m_ = std::cbrt(m);
		const float s_ = std::cbrt(s);

		return {
			 0.2104542553f * l_ + 0.7936177850f * m_ - 0.0040720468f * s_,
			 1.9779984951f * l_ - 2.4285922050f * m_ + 0.4505937099f * s_,
			 0.0259040371f * l_ + 0.7827717662f * m_ - 0.8086757660f * s_
		};
	}

	inline float3 oklabToLinearRgb(const float3& lab)
	{
		const float l_ = lab.x + 0.3963377774f * lab.y + 0.2158037573f * lab.z;
		const float m_ = lab.x - 0.1055613458f * lab.y - 0.0638541728f * lab.z;
		const float s_ = lab.x - 0.0894841775f * lab.y - 1.2914855480f * lab.z;

		const float l = l_ * l_ * l_;
		const float m = m_ * m_ * m_;
		const float s = s_ * s_ * s_;

		return {
			 4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s,
			-1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s,
			-0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s
		};
	}

	inline float3 nlRgbToOklab(const float3& c)
	{
		return linearRgbToOklab(srgbNlToLinear3(c));
	}

	// ---- Gamut clamp ------------------------------------------------------
	// The chroma boost can push a color out of the sRGB gamut. The naive
	// per-channel clamp inside the Oklab->sRGB conversion would then shift
	// the hue (e.g. a dark red becomes a pure saturated red, brown becomes
	// orange, etc.). To avoid that we binary-search the maximum chroma
	// that still fits in sRGB before doing the conversion.
	//
	// 5 iterations are enough for visual purposes: the error is well below
	// one 8-bit RGB step.

	inline bool isInsideGamut(const float3& lab)
	{
		const float3 lin = oklabToLinearRgb(lab);
		return lin.x >= 0.0f && lin.x <= 1.0f
		    && lin.y >= 0.0f && lin.y <= 1.0f
		    && lin.z >= 0.0f && lin.z <= 1.0f;
	}

	inline float3 clampChromaToGamut(const float3& lab)
	{
		if (isInsideGamut(lab))
			return lab;

		const float L = lab.x;
		const float a = lab.y;
		const float b = lab.z;

		const float chroma = std::sqrt(a * a + b * b);
		if (chroma < 1e-5f)
			return lab;

		const float dirA = a / chroma;
		const float dirB = b / chroma;

		float lo = 0.0f;
		float hi = chroma;

		for (int i = 0; i < 5; ++i)
		{
			const float mid = 0.5f * (lo + hi);
			const float3 test{ L, dirA * mid, dirB * mid };
			if (isInsideGamut(test))
				lo = mid;
			else
				hi = mid;
		}

		return { L, dirA * lo, dirB * lo };
	}

	inline float3 oklabToNlRgbSafe(const float3& lab)
	{
		return srgbLinearToNl3(linalg::clamp(oklabToLinearRgb(lab), 0.0f, 1.0f));
	}

	// 3-tap spatial smoothing along the strip
	void spatialSmoothOklab(std::vector<float3>& colors, float strength)
	{
		if (strength <= 0.0f || colors.size() < 3)
			return;

		const size_t n = colors.size();
		std::vector<float3> tmp(n);

		const float s    = strength;
		const float half = s * 0.5f;

		tmp[0]     = (1.0f - s) * colors[0]     + s * colors[1];
		tmp[n - 1] = (1.0f - s) * colors[n - 1] + s * colors[n - 2];

		for (size_t i = 1; i + 1 < n; ++i)
		{
			tmp[i] = (1.0f - s) * colors[i]
			       + half * (colors[i - 1] + colors[i + 1]);
		}

		colors.swap(tmp);
	}
}

// ===========================================================================

InfiniteKineticInterpolator::InfiniteKineticInterpolator() = default;

void InfiniteKineticInterpolator::setTransitionDuration(float durationMs)
{
	_halfLifeMs = std::max(5.0f, durationMs * 0.5f);
}

void InfiniteKineticInterpolator::setSpringiness(float /*stiffness*/, float /*damping*/)
{
	// Not used: this interpolator uses EMA, not spring physics.
}

void InfiniteKineticInterpolator::setSmoothingFactor(float factor)
{
	// Repurposed as scene-change sensitivity.
	_sceneSensitivity = std::clamp(factor, 0.0f, 1.0f);
}

void InfiniteKineticInterpolator::setChromaBoost(float boost)
{
	_chromaBoost = std::clamp(boost, 0.0f, 0.5f);
}

void InfiniteKineticInterpolator::setSpatialStrength(float strength)
{
	_spatialStrength = std::clamp(strength, 0.0f, 0.45f);
}

void InfiniteKineticInterpolator::resetState()
{
	_isAnimationComplete = true;
	_lastUpdate = 0;
	_targetOklab.clear();
	_currentOklab.clear();
	_prevTargetOklab.clear();
	_outputRgb.clear();
	_sceneChangeSmoothed = 0.0f;
}

void InfiniteKineticInterpolator::resetToColors(std::vector<float3> colors, long long startTimeMs)
{
	if (colors.empty())
	{
		_isAnimationComplete = true;
		return;
	}

	for (auto& c : colors)
		c = nlRgbToOklab(c);

	_currentOklab    = colors;
	_targetOklab     = std::move(colors);
	_prevTargetOklab = _targetOklab;

	_isAnimationComplete = true;
	_lastUpdate = startTimeMs;
	_outputRgb.clear();
	_sceneChangeSmoothed = 0.0f;
}

void InfiniteKineticInterpolator::setTargetColors(std::vector<float3>&& new_rgb_targets,
                                                  long long startTimeMs, bool /*debug*/)
{
	if (new_rgb_targets.empty())
		return;

	// 1. Convert to Oklab
	std::vector<float3> targets(new_rgb_targets.size());
	std::transform(new_rgb_targets.begin(), new_rgb_targets.end(),
	               targets.begin(),
	               [](const float3& c) { return nlRgbToOklab(c); });

	// 2. Detect scene change BEFORE smoothing
	float globalChange = 0.0f;
	if (_prevTargetOklab.size() == targets.size())
	{
		for (size_t i = 0; i < targets.size(); ++i)
		{
			const float3 d = targets[i] - _prevTargetOklab[i];
			globalChange += std::sqrt(linalg::dot(d, d));
		}
		globalChange /= static_cast<float>(targets.size());
	}
	_prevTargetOklab = targets;

	// Smooth the global change measure
	_sceneChangeSmoothed = _sceneChangeSmoothed * 0.6f + globalChange * 0.4f;

	// 3. Content-aware spatial smoothing
	const float sceneFactor = std::clamp(_sceneChangeSmoothed * 8.0f, 0.0f, 1.0f);
	const float effectiveSpatial = _spatialStrength * (1.0f - sceneFactor * 0.7f);

	if (effectiveSpatial > 0.0f)
	{
		spatialSmoothOklab(targets, effectiveSpatial);
	}

	// 4. Chroma boost (after spatial, before temporal).
	//    We gamut-clamp here so the temporal integration always works with
	//    valid sRGB-representable colors. This is what prevents the
	//    "dark color becomes pure red" artifact.
	if (_chromaBoost > 0.0f)
	{
		const float scale = 1.0f + _chromaBoost;
		for (auto& lab : targets)
		{
			lab.y *= scale;
			lab.z *= scale;
			lab = clampChromaToGamut(lab);
		}
	}

	if (_currentOklab.size() != targets.size())
	{
		_currentOklab = targets;
		_targetOklab  = std::move(targets);
		_isAnimationComplete = true;
	}
	else
	{
		_targetOklab = std::move(targets);
		_isAnimationComplete = false;
	}

	_lastUpdate = startTimeMs;
	_outputRgb.clear();
}

void InfiniteKineticInterpolator::updateCurrentColors(long long currentTimeMs, float /*minBrightness*/)
{
	if (_isAnimationComplete)
	{
		_lastUpdate = currentTimeMs;
		return;
	}

	const float dt = std::clamp(static_cast<float>(currentTimeMs - _lastUpdate), 0.001f, 100.0f);
	_lastUpdate = currentTimeMs;

	// Base EMA coefficient (release speed)
	const float baseCoeff = 1.0f - std::pow(0.5f, dt / std::max(_halfLifeMs, 1.0f));

	// Scene-change boost
	const float sceneBoost = std::clamp(_sceneChangeSmoothed * 10.0f, 0.0f, 1.0f)
	                       * _sceneSensitivity;

	const float attackCoeff = std::min(1.0f, baseCoeff + sceneBoost * 0.8f);

	_isAnimationComplete = true;

	for (size_t i = 0; i < _currentOklab.size(); ++i)
	{
		float3&       cur = _currentOklab[i];
		const float3& tgt = _targetOklab[i];

		const float3 diff = tgt - cur;
		const float  mag  = linalg::maxelem(linalg::abs(diff));

		// Per-LED magnitude-adaptive attack
		const float magFactor = std::clamp(mag * 8.0f, 0.0f, 1.0f);
		float coeff = baseCoeff + (attackCoeff - baseCoeff) * magFactor;

		// Hard cut: snap immediately
		if (mag > 0.5f)
		{
			coeff = 1.0f;
		}

		cur.x += diff.x * coeff;
		cur.y += diff.y * coeff;
		cur.z += diff.z * coeff;

		if (mag > 0.002f)
		{
			_isAnimationComplete = false;
		}
	}

	_outputRgb.clear();
}

SharedOutputColors InfiniteKineticInterpolator::getCurrentColors(float minBrightness)
{
	if (_outputRgb.empty() && !_currentOklab.empty())
	{
		_outputRgb.resize(_currentOklab.size());

		for (size_t i = 0; i < _currentOklab.size(); ++i)
		{
			// Gamut clamp the current color too. Even though setTargetColors
			// already clamped the target, the temporal integration can still
			// produce intermediate values that momentarily exceed the gamut.
			// This is the last line of defense against the "hue shift on dark
			// colors" bug.
			const float3 lab = clampChromaToGamut(_currentOklab[i]);

			float3 rgb = oklabToNlRgbSafe(lab);

			if (minBrightness > 0.0f)
			{
				rgb = linalg::clamp(rgb, minBrightness, 1.0f);
			}

			_outputRgb[i] = rgb;
		}
	}

	return std::make_shared<std::vector<float3>>(_outputRgb);
}

void InfiniteKineticInterpolator::test()
{
	InfiniteKineticInterpolator interp;
	interp.setTransitionDuration(150.0f);
	interp.setSmoothingFactor(0.5f);
	interp.setChromaBoost(0.20f);
	interp.setSpatialStrength(0.20f);

	std::vector<float3> black(4, { 0, 0, 0 });
	interp.resetToColors(black, 0);

	std::vector<float3> colors = {
		{ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 1, 1, 0 }
	};
	interp.setTargetColors(std::move(colors), 0);

	std::cout << "Kinetic interpolator test:\n";
	for (long long t = 0; t <= 600; t += 50)
	{
		interp.updateCurrentColors(t, 0.0f);
		auto c = interp.getCurrentColors();
		std::cout << std::setw(4) << t << "ms: ";
		for (size_t i = 0; i < c->size(); ++i)
		{
			const auto& v = (*c)[i];
			std::cout << std::fixed << std::setprecision(2)
			          << "(" << v.x << "," << v.y << "," << v.z << ") ";
		}
		std::cout << "\n";
	}
}