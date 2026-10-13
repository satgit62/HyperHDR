/* InfiniteAdaptiveEmaInterpolator.cpp
 *
 *  MIT License
 *  Copyright (c) 2020-2026 awawa-dev
 *
 *  Adaptive EMA smoothing with scene-cut detection.
 */

#ifndef PCH_ENABLED
	#include <cmath>
	#include <algorithm>
	#include <vector>
	#include <iostream>
	#include <iomanip>
#endif

#include <infinite-color-engine/InfiniteAdaptiveEmaInterpolator.h>

using namespace linalg::aliases;

InfiniteAdaptiveEmaInterpolator::InfiniteAdaptiveEmaInterpolator() = default;

void InfiniteAdaptiveEmaInterpolator::setTransitionDuration(float durationMs)
{
	_halfLifeMs = std::max(5.0f, durationMs * 0.4f);
}

void InfiniteAdaptiveEmaInterpolator::setSmoothingFactor(float factor)
{
	factor = std::clamp(factor, 0.0f, 1.0f);
	_triggerDelta = 0.05f + factor * 0.35f;
}

void InfiniteAdaptiveEmaInterpolator::setSpringiness(float /*stiffness*/, float /*damping*/) {}

void InfiniteAdaptiveEmaInterpolator::resetState()
{
	_isAnimationComplete = true;
	_lastUpdate = 0;
	_current.clear();
	_target.clear();
}

void InfiniteAdaptiveEmaInterpolator::resetToColors(std::vector<float3> colors, long long startTimeMs)
{
	if (colors.empty()) { _isAnimationComplete = true; return; }
	_current = colors;
	_target  = std::move(colors);
	_isAnimationComplete = true;
	_lastUpdate = startTimeMs;
}

void InfiniteAdaptiveEmaInterpolator::setTargetColors(std::vector<float3>&& new_rgb_targets,
                                                     long long startTimeMs, bool /*debug*/)
{
	if (new_rgb_targets.empty())
		return;

	if (_current.size() != new_rgb_targets.size())
	{
		_current = new_rgb_targets;
		_target  = std::move(new_rgb_targets);
		_isAnimationComplete = true;
		_lastUpdate = startTimeMs;
		return;
	}
	_target = std::move(new_rgb_targets);
	_isAnimationComplete = false;
}

void InfiniteAdaptiveEmaInterpolator::updateCurrentColors(long long currentTimeMs, float /*minBrightness*/)
{
	if (_isAnimationComplete) { _lastUpdate = currentTimeMs; return; }

	const float dt = std::clamp(static_cast<float>(currentTimeMs - _lastUpdate), 0.001f, 100.0f);
	_lastUpdate = currentTimeMs;

	const float baseAlpha = 1.0f - std::pow(0.5f, dt / std::max(_halfLifeMs, 1.0f));

	_isAnimationComplete = true;

	for (size_t i = 0; i < _current.size(); ++i)
	{
		const float3 diff = _target[i] - _current[i];
		const float mag   = linalg::maxelem(linalg::abs(diff));

		float alpha = baseAlpha;
		if (mag > _triggerDelta)
			alpha = std::min(1.0f, baseAlpha + (mag - _triggerDelta) * 4.0f);

		_current[i] += diff * alpha;

		if (mag > 0.0015f)
			_isAnimationComplete = false;
	}
}

SharedOutputColors InfiniteAdaptiveEmaInterpolator::getCurrentColors(float minBrightness)
{
	if (minBrightness <= 0.0f)
		return std::make_shared<std::vector<float3>>(_current);

	auto out = _current;
	for (auto& c : out) c = linalg::clamp(c, minBrightness, 1.0f);
	return std::make_shared<std::vector<float3>>(std::move(out));
}

void InfiniteAdaptiveEmaInterpolator::test()
{
	InfiniteAdaptiveEmaInterpolator interp;
	interp.setTransitionDuration(150.0f);
	interp.setSmoothingFactor(0.3f);
	interp.resetToColors({ {0,0,0} }, 0);
	interp.setTargetColors({ {1,0,0} }, 0);

	std::cout << "Adaptive EMA test:\n";
	for (long long t = 0; t <= 500; t += 50)
	{
		interp.updateCurrentColors(t, 0.0f);
		auto c = interp.getCurrentColors();
		std::cout << std::setw(4) << t << "ms: "
		          << std::fixed << std::setprecision(3)
		          << (*c)[0].x << " " << (*c)[0].y << " " << (*c)[0].z << "\n";
	}
}