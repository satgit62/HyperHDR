/* InfiniteFluidInterpolator.cpp
 *
 *  MIT License
 *  Copyright (c) 2020-2026 awawa-dev
 */

#include <infinite-color-engine/InfiniteFluidInterpolator.h>
#include <cmath>
#include <algorithm>

using namespace linalg::aliases;

namespace
{
	// Fast cube root approximation (same as in ImageColorAveraging)
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

	inline float srgbNlToLinear(float v)
	{
		v = std::clamp(v, 0.0f, 1.0f);
		return (v <= 0.04045f) ? (v / 12.92f) : std::pow((v + 0.055f) / 1.055f, 2.4f);
	}
	
	inline float srgbLinearToNl(float v)
	{
		v = std::clamp(v, 0.0f, 1.0f);
		return (v <= 0.0031308f) ? (v * 12.92f) : (1.055f * std::pow(v, 1.0f / 2.4f) - 0.055f);
	}

	inline float3 nlRgbToOklab(const float3& c)
	{
		const float r = srgbNlToLinear(c.x);
		const float g = srgbNlToLinear(c.y);
		const float b = srgbNlToLinear(c.z);
		const float l = 0.4122214708f * r + 0.5363325363f * g + 0.0514459929f * b;
		const float m = 0.2119034982f * r + 0.6806995451f * g + 0.1073969566f * b;
		const float s = 0.0883024619f * r + 0.2817188376f * g + 0.6299787005f * b;
		const float l_ = fast_cbrt(l), m_ = fast_cbrt(m), s_ = fast_cbrt(s);
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
		const float l = l_ * l_ * l_, m = m_ * m_ * m_, s = s_ * s_ * s_;
		return {
			 4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s,
			-1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s,
			-0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s
		};
	}

	inline bool isInsideGamut(const float3& lab)
	{
		const float3 lin = oklabToLinearRgb(lab);
		return lin.x >= 0.0f && lin.x <= 1.0f && lin.y >= 0.0f && lin.y <= 1.0f && lin.z >= 0.0f && lin.z <= 1.0f;
	}

	inline float3 clampChromaToGamut(const float3& lab)
	{
		if (isInsideGamut(lab)) return lab;
		const float L = lab.x, a = lab.y, b = lab.z;
		const float chroma = std::sqrt(a * a + b * b);
		if (chroma < 1e-5f) return lab;
		const float dirA = a / chroma, dirB = b / chroma;
		float lo = 0.0f, hi = chroma;
		for (int i = 0; i < 5; ++i) {
			const float mid = 0.5f * (lo + hi);
			if (isInsideGamut({ L, dirA * mid, dirB * mid })) lo = mid;
			else hi = mid;
		}
		return { L, dirA * lo, dirB * lo };
	}

	inline float3 oklabToNlRgbSafe(const float3& lab)
	{
		const float3 lin = oklabToLinearRgb(lab);
		return {
			srgbLinearToNl(std::clamp(lin.x, 0.0f, 1.0f)),
			srgbLinearToNl(std::clamp(lin.y, 0.0f, 1.0f)),
			srgbLinearToNl(std::clamp(lin.z, 0.0f, 1.0f))
		};
	}

	// Multi-pass spatial blur in Oklab space (Gaussian-like approximation)
	void applyStrongSpatialBlur(std::vector<float3>& colors, float strength, int iterations)
	{
		if (strength <= 0.0f || colors.size() < 3) return;
		
		const size_t n = colors.size();
		std::vector<float3> tmp(n);
		const float half = 0.5f * strength;

		for (int iter = 0; iter < iterations; ++iter) {
			tmp[0]     = (1.0f - strength) * colors[0]     + strength * colors[1];
			tmp[n - 1] = (1.0f - strength) * colors[n - 1] + strength * colors[n - 2];
			
			for (size_t i = 1; i + 1 < n; ++i) {
				tmp[i] = (1.0f - strength) * colors[i] + half * (colors[i - 1] + colors[i + 1]);
			}
			colors.swap(tmp);
		}
	}
}

InfiniteFluidInterpolator::InfiniteFluidInterpolator() = default;

void InfiniteFluidInterpolator::setTransitionDuration(float durationMs)
{
	_halfLifeMs = std::max(5.0f, durationMs * 0.6f);
}

void InfiniteFluidInterpolator::setSpringiness(float /*stiffness*/, float /*damping*/)
{
	// Not used: this interpolator uses EMA, not spring physics.
}

void InfiniteFluidInterpolator::setSmoothingFactor(float /*factor*/)
{
	// Not used: temporal smoothing is controlled by the transition duration.
}

void InfiniteFluidInterpolator::setMaxLuminanceChangePerFrame(float /*maxChangePerStep*/)
{
	// Not used.
}

void InfiniteFluidInterpolator::setSpatialStrength(float strength)
{
	_spatialStrength = std::clamp(strength, 0.0f, 0.45f);
}

void InfiniteFluidInterpolator::resetState()
{
	_isAnimationComplete = true;
	_lastUpdate = 0;
	_targetOklab.clear();
	_currentOklab.clear();
	_outputRgb.clear();
}

void InfiniteFluidInterpolator::resetToColors(std::vector<float3> colors, long long startTimeMs)
{
	if (colors.empty()) { _isAnimationComplete = true; return; }

	for (auto& c : colors) c = nlRgbToOklab(c);
	applyStrongSpatialBlur(colors, _spatialStrength, 3);
	for (auto& lab : colors) lab = clampChromaToGamut(lab);

	_currentOklab = colors;
	_targetOklab = std::move(colors);
	_isAnimationComplete = true;
	_lastUpdate = startTimeMs;
}

void InfiniteFluidInterpolator::setTargetColors(std::vector<float3>&& new_rgb_targets, long long startTimeMs, bool /*debug*/)
{
	if (new_rgb_targets.empty()) return;

	std::vector<float3> targets(new_rgb_targets.size());
	for (size_t i = 0; i < new_rgb_targets.size(); ++i)
		targets[i] = nlRgbToOklab(new_rgb_targets[i]);

	// Apply spatial blur to the targets before temporal integration.
	// This is the key to smooth transitions between zones.
	applyStrongSpatialBlur(targets, _spatialStrength, 3);
	for (auto& lab : targets) lab = clampChromaToGamut(lab);

	if (_currentOklab.size() != targets.size())
	{
		_currentOklab = targets;
		_targetOklab = std::move(targets);
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

void InfiniteFluidInterpolator::updateCurrentColors(long long currentTimeMs, float /*minBrightness*/)
{
	if (_isAnimationComplete) { _lastUpdate = currentTimeMs; return; }

	const float dt = std::clamp(static_cast<float>(currentTimeMs - _lastUpdate), 0.001f, 100.0f);
	_lastUpdate = currentTimeMs;

	const float alpha = 1.0f - std::pow(0.5f, dt / std::max(_halfLifeMs, 1.0f));

	_isAnimationComplete = true;
	for (size_t i = 0; i < _currentOklab.size(); ++i)
	{
		const float3 diff = _targetOklab[i] - _currentOklab[i];
		_currentOklab[i] += diff * alpha;

		if (linalg::maxelem(linalg::abs(diff)) > 0.002f)
			_isAnimationComplete = false;
	}

	_outputRgb.clear();
}

SharedOutputColors InfiniteFluidInterpolator::getCurrentColors(float minBrightness)
{
	if (_outputRgb.empty() && !_currentOklab.empty())
	{
		std::vector<float3> outColors = _currentOklab;
		
		// Final spatial smoothing pass on the output for perfect continuity
		applyStrongSpatialBlur(outColors, _spatialStrength, 2);

		_outputRgb.resize(outColors.size());
		for (size_t i = 0; i < outColors.size(); ++i)
		{
			float3 lab = clampChromaToGamut(outColors[i]);
			_outputRgb[i] = oklabToNlRgbSafe(lab);
			
			if (minBrightness > 0.0f)
				_outputRgb[i] = linalg::clamp(_outputRgb[i], minBrightness, 1.0f);
		}
	}
	return std::make_shared<std::vector<float3>>(_outputRgb);
}