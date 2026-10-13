/* InfinitePerceptualInterpolator.cpp
 *
 *  MIT License
 *  Copyright (c) 2020-2026 awawa-dev
 *
 *  Oklab perceptual interpolator with optional spatial smoothing.
 *  Input is nonlinear sRGB, so we do:
 *     nonlinear sRGB -> linear RGB -> Oklab
 *     [spring physics + spatial smooth in Oklab]
 *     Oklab -> linear RGB -> nonlinear sRGB
 */

#ifndef PCH_ENABLED
	#include <cmath>
	#include <algorithm>
	#include <vector>
	#include <cstdint>
	#include <iostream>
	#include <iomanip>
#endif

#include <infinite-color-engine/InfinitePerceptualInterpolator.h>

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

	// ---- Oklab (Ottosson 2020) -------------------------------------------
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

	inline float3 oklabToNlRgb(const float3& lab)
	{
		return srgbLinearToNl3(linalg::clamp(oklabToLinearRgb(lab), 0.0f, 1.0f));
	}

	// 3-tap spatial smoothing in Oklab along the strip.
	// strength 0.0 = off, ~0.45 = strong (edges use a single neighbour).
	void spatialSmoothOklab(std::vector<float3>& colors, float strength)
	{
		if (strength <= 0.0f || colors.size() < 3)
			return;

		const size_t n = colors.size();
		std::vector<float3> tmp(n);

		tmp[0]     = (1.0f - strength) * colors[0]     + strength * colors[1];
		tmp[n - 1] = (1.0f - strength) * colors[n - 1] + strength * colors[n - 2];

		const float half = 0.5f * strength;
		for (size_t i = 1; i + 1 < n; ++i)
		{
			tmp[i] = (1.0f - strength) * colors[i]
			       + half * (colors[i - 1] + colors[i + 1]);
		}
		colors.swap(tmp);
	}
}

// ---------------------------------------------------------------------------

InfinitePerceptualInterpolator::InfinitePerceptualInterpolator() = default;

void InfinitePerceptualInterpolator::setTransitionDuration(float durationMs)
{
	_initialDuration = std::max(1.0f, durationMs);
}

void InfinitePerceptualInterpolator::setSpringiness(float stiffness, float damping)
{
	_stiffness = std::max(0.1f, stiffness);
	_damping   = std::max(0.1f, damping);
}

void InfinitePerceptualInterpolator::setSmoothingFactor(float factor)
{
	_smoothingFactor = std::clamp(factor, 0.0f, 1.0f);
}

void InfinitePerceptualInterpolator::setSpatialSmoothing(float strength)
{
	_spatialSmoothing = std::clamp(strength, 0.0f, 0.45f);
}

void InfinitePerceptualInterpolator::resetState()
{
	_isAnimationComplete = true;
	_lastUpdate = 0;
	_startAnimationTimeMs = 0;
	_targetTime = 0;
	_targetColorsOklab.clear();
	_currentColorsOklab.clear();
	_velocitiesOklab.clear();
	_currentColorsRGB.clear();
}

void InfinitePerceptualInterpolator::resetToColors(std::vector<float3> colors, long long startTimeMs)
{
	if (colors.empty())
	{
		_isAnimationComplete = true;
		return;
	}

	for (auto& c : colors)
		c = nlRgbToOklab(c);
	spatialSmoothOklab(colors, _spatialSmoothing);

	_currentColorsOklab = colors;
	_targetColorsOklab  = std::move(colors);
	_velocitiesOklab.assign(_currentColorsOklab.size(), float3{ 0, 0, 0 });
	_isAnimationComplete = true;

	_lastUpdate = startTimeMs;
	_startAnimationTimeMs = startTimeMs;
	_targetTime = startTimeMs + static_cast<long long>(_initialDuration);
	_currentColorsRGB.clear();
}

void InfinitePerceptualInterpolator::setTargetColors(std::vector<float3>&& new_rgb_targets,
                                                     long long startTimeMs, bool debug)
{
	if (new_rgb_targets.empty())
		return;

	// nonlinear sRGB -> Oklab
	std::vector<float3> new_targets_oklab(new_rgb_targets.size());
	std::transform(new_rgb_targets.begin(), new_rgb_targets.end(),
	               new_targets_oklab.begin(),
	               [](const float3& c) { return nlRgbToOklab(c); });

	// Spatial smoothing on the target itself (keeps the visual center of mass stable)
	spatialSmoothOklab(new_targets_oklab, _spatialSmoothing);

	const float delta = (!_isAnimationComplete)
		? std::clamp(static_cast<float>(startTimeMs - _lastUpdate), 0.0f, 100.0f)
		: 0.0f;

	if (debug)
	{
		printf(" | d%.0f | time: %.0f | smooth: %.3f | stiff: %.3f | damp: %.3f | spatial: %.3f\n",
		       delta, _initialDuration, _smoothingFactor, _stiffness, _damping, _spatialSmoothing);
	}

	startTimeMs -= static_cast<long long>(delta);

	if (_currentColorsOklab.size() != new_targets_oklab.size())
	{
		_lastUpdate         = startTimeMs;
		_currentColorsOklab = new_targets_oklab;
		_targetColorsOklab  = std::move(new_targets_oklab);
		_velocitiesOklab.assign(_currentColorsOklab.size(), float3{ 0, 0, 0 });
		_isAnimationComplete = true;
	}
	else
	{
		if (_smoothingFactor > 0.0f)
		{
			const float inv = 1.0f - _smoothingFactor;
			for (auto itOld = _targetColorsOklab.begin(),
			          itNew = new_targets_oklab.begin();
			     itOld != _targetColorsOklab.end();
			     ++itOld, ++itNew)
			{
				*itOld = *itOld * _smoothingFactor + *itNew * inv;
			}
		}
		else
		{
			_targetColorsOklab = std::move(new_targets_oklab);
		}
		_isAnimationComplete = false;
	}

	_startAnimationTimeMs = startTimeMs;
	_targetTime = startTimeMs + static_cast<long long>(_initialDuration);
	_currentColorsRGB.clear();
}

void InfinitePerceptualInterpolator::updateCurrentColors(long long currentTimeMs, float /*minBrightness*/)
{
	if (_isAnimationComplete)
	{
		_lastUpdate = currentTimeMs;
		return;
	}

	const float dt = std::clamp(static_cast<float>(currentTimeMs - _lastUpdate), 0.001f, 100.0f);
	_lastUpdate = currentTimeMs;

	constexpr float L_MIN  = 0.0f,  L_MAX  = 1.0f;
	constexpr float AB_MIN = -0.5f, AB_MAX = 0.5f;

	auto stepChannel = [&](float3& cur, const float3& tgt, float3& vel) -> bool
	{
		constexpr float FINISH_COMPONENT_THRESHOLD = 0.0013732906f / 10.0f;
		constexpr float VELOCITY_THRESHOLD         = 0.0005f;

		const float3 diff = tgt - cur;

		if (linalg::maxelem(linalg::abs(diff)) < FINISH_COMPONENT_THRESHOLD &&
		    linalg::maxelem(linalg::abs(vel))  < VELOCITY_THRESHOLD)
		{
			cur = tgt;
			vel = float3{ 0, 0, 0 };
			return false;
		}

		const float3 acc = _stiffness * diff - _damping * vel;

		// Semi-implicit Euler
		vel += acc * (dt * 0.001f);
		cur += vel * (dt * 0.001f);

		// Clamp to the Oklab gamut envelope
		if (cur.x < L_MIN)  { cur.x = L_MIN;  vel.x = 0.f; }
		if (cur.x > L_MAX)  { cur.x = L_MAX;  vel.x = 0.f; }
		if (cur.y < AB_MIN) { cur.y = AB_MIN; vel.y = 0.f; }
		if (cur.y > AB_MAX) { cur.y = AB_MAX; vel.y = 0.f; }
		if (cur.z < AB_MIN) { cur.z = AB_MIN; vel.z = 0.f; }
		if (cur.z > AB_MAX) { cur.z = AB_MAX; vel.z = 0.f; }

		return true;
	};

	_isAnimationComplete = true;

	for (auto cur = _currentColorsOklab.begin(),
	          tgt = _targetColorsOklab.begin(),
	          vel = _velocitiesOklab.begin();
	     cur != _currentColorsOklab.end() &&
	     tgt != _targetColorsOklab.end() &&
	     vel != _velocitiesOklab.end();
	     ++cur, ++tgt, ++vel)
	{
		if (stepChannel(*cur, *tgt, *vel))
			_isAnimationComplete = false;
	}

	_currentColorsRGB.clear();
}

SharedOutputColors InfinitePerceptualInterpolator::getCurrentColors(float minBrightness)
{
	if (_currentColorsRGB.empty() && !_currentColorsOklab.empty())
	{
		_currentColorsRGB.resize(_currentColorsOklab.size());
		std::transform(_currentColorsOklab.begin(), _currentColorsOklab.end(),
		               _currentColorsRGB.begin(),
		               [](const float3& lab) { return oklabToNlRgb(lab); });

		if (minBrightness > 0.0f)
		{
			for (auto& c : _currentColorsRGB)
				c = linalg::clamp(c, minBrightness, 1.0f);
		}
	}
	return std::make_shared<std::vector<float3>>(_currentColorsRGB);
}

void InfinitePerceptualInterpolator::test()
{
	InfinitePerceptualInterpolator interp;
	interp.setSpringiness(200.0f, 26.0f);
	interp.setSpatialSmoothing(0.0f);
	interp.resetToColors({ {0,0,0} }, 0);
	interp.setTargetColors({ {1,0,0} }, 0);

	std::cout << "Perceptual (Oklab) test:\n";
	for (long long t = 0; t <= 500; t += 50)
	{
		interp.updateCurrentColors(t, 0.0f);
		auto c = interp.getCurrentColors();
		std::cout << std::setw(4) << t << "ms: "
		          << std::fixed << std::setprecision(3)
		          << (*c)[0].x << " " << (*c)[0].y << " " << (*c)[0].z << "\n";
	}
}