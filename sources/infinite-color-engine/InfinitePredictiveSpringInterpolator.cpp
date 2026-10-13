/* InfinitePredictiveSpringInterpolator.cpp
 *
 *  MIT License
 *  Copyright (c) 2020-2026 awawa-dev
 *
 *  Spring physics with a velocity-based lookahead term.
 */

#ifndef PCH_ENABLED
	#include <cmath>
	#include <algorithm>
	#include <vector>
	#include <iostream>
	#include <iomanip>
#endif

#include <infinite-color-engine/InfinitePredictiveSpringInterpolator.h>

using namespace linalg::aliases;

InfinitePredictiveSpringInterpolator::InfinitePredictiveSpringInterpolator() = default;

void InfinitePredictiveSpringInterpolator::setTransitionDuration(float durationMs)
{
	_stiffness = std::clamp(20000.0f / std::max(durationMs, 20.0f), 30.0f, 600.0f);
	_damping   = std::clamp(2.0f * std::sqrt(_stiffness) * 1.3f, 10.0f, 60.0f);
}

void InfinitePredictiveSpringInterpolator::setSpringiness(float stiffness, float damping)
{
	_stiffness = std::max(0.1f, stiffness);
	_damping   = std::max(0.1f, damping);
}

void InfinitePredictiveSpringInterpolator::resetState()
{
	_isAnimationComplete = true;
	_lastUpdate = 0;
	_lastTargetT = 0;
	_current.clear(); _target.clear();
	_vel.clear();     _targetVel.clear();
}

void InfinitePredictiveSpringInterpolator::resetToColors(std::vector<float3> colors, long long startTimeMs)
{
	if (colors.empty()) { _isAnimationComplete = true; return; }

	_current   = colors;
	_target    = std::move(colors);
	_vel.assign(_current.size(), float3{ 0, 0, 0 });
	_targetVel.assign(_current.size(), float3{ 0, 0, 0 });

	_isAnimationComplete = true;
	_lastUpdate  = startTimeMs;
	_lastTargetT = startTimeMs;
}

void InfinitePredictiveSpringInterpolator::setTargetColors(std::vector<float3>&& new_rgb_targets,
                                                           long long startTimeMs, bool /*debug*/)
{
	if (new_rgb_targets.empty()) return;

	if (_current.size() != new_rgb_targets.size())
	{
		resetToColors(new_rgb_targets, startTimeMs);
		return;
	}

	const float dt = std::clamp(static_cast<float>(startTimeMs - _lastTargetT), 0.001f, 100.0f);

	for (size_t i = 0; i < new_rgb_targets.size(); ++i)
	{
		const float3 instVel = (new_rgb_targets[i] - _target[i]) * (1000.0f / dt);
		_targetVel[i] = _targetVel[i] * 0.7f + instVel * 0.3f;
	}

	_target      = std::move(new_rgb_targets);
	_lastTargetT = startTimeMs;
	_isAnimationComplete = false;
}

void InfinitePredictiveSpringInterpolator::updateCurrentColors(long long currentTimeMs, float /*minBrightness*/)
{
	if (_isAnimationComplete || _current.empty()) { _lastUpdate = currentTimeMs; return; }

	const float dtMs = std::clamp(static_cast<float>(currentTimeMs - _lastUpdate), 0.001f, 100.0f);
	_lastUpdate = currentTimeMs;
	const float dtSec = dtMs * 0.001f;

	_isAnimationComplete = true;

	for (size_t i = 0; i < _current.size(); ++i)
	{
		const float3 predicted = linalg::clamp(
			_target[i] + _targetVel[i] * _lookaheadSec, 0.0f, 1.0f);

		const float3 diff = predicted - _current[i];
		const float3 acc  = _stiffness * diff - _damping * _vel[i];

		_vel[i]     += acc * dtSec;
		_current[i] += _vel[i] * dtSec;

		for (int k = 0; k < 3; ++k)
		{
			if (_current[i][k] < 0.0f) { _current[i][k] = 0.0f; _vel[i][k] = 0.0f; }
			if (_current[i][k] > 1.0f) { _current[i][k] = 1.0f; _vel[i][k] = 0.0f; }
		}

		if (linalg::maxelem(linalg::abs(_target[i] - _current[i])) > 0.0015f)
			_isAnimationComplete = false;
	}
}

SharedOutputColors InfinitePredictiveSpringInterpolator::getCurrentColors(float minBrightness)
{
	if (minBrightness <= 0.0f)
		return std::make_shared<std::vector<float3>>(_current);

	auto out = _current;
	for (auto& c : out) c = linalg::clamp(c, minBrightness, 1.0f);
	return std::make_shared<std::vector<float3>>(std::move(out));
}

void InfinitePredictiveSpringInterpolator::test()
{
	InfinitePredictiveSpringInterpolator interp;
	interp.setTransitionDuration(150.0f);
	interp.resetToColors({ {0,0,0} }, 0);
	interp.setTargetColors({ {1,0,0} }, 0);

	std::cout << "Predictive Spring test:\n";
	for (long long t = 0; t <= 500; t += 50)
	{
		interp.updateCurrentColors(t, 0.0f);
		auto c = interp.getCurrentColors();
		std::cout << std::setw(4) << t << "ms: "
		          << std::fixed << std::setprecision(3)
		          << (*c)[0].x << " " << (*c)[0].y << " " << (*c)[0].z << "\n";
	}
}