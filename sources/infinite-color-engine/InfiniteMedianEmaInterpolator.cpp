/* InfiniteMedianEmaInterpolator.cpp
 *
 *  MIT License
 *  Copyright (c) 2020-2026 awawa-dev
 *
 *  Median-of-3 spike rejection followed by EMA smoothing.
 */

#ifndef PCH_ENABLED
	#include <cmath>
	#include <algorithm>
	#include <vector>
	#include <iostream>
	#include <iomanip>
#endif

#include <infinite-color-engine/InfiniteMedianEmaInterpolator.h>

using namespace linalg::aliases;

InfiniteMedianEmaInterpolator::InfiniteMedianEmaInterpolator() = default;

void InfiniteMedianEmaInterpolator::setTransitionDuration(float durationMs)
{
	_halfLifeMs = std::max(5.0f, durationMs * 0.45f);
}

void InfiniteMedianEmaInterpolator::setSmoothingFactor(float factor)
{
	factor = std::clamp(factor, 0.0f, 1.0f);
	_halfLifeMs *= (1.0f + factor);
}

void InfiniteMedianEmaInterpolator::resetState()
{
	_isAnimationComplete = true;
	_lastUpdate = 0;
	_buf0.clear(); _buf1.clear(); _buf2.clear();
	_median.clear(); _current.clear(); _target.clear();
}

void InfiniteMedianEmaInterpolator::resetToColors(std::vector<float3> colors, long long startTimeMs)
{
	if (colors.empty()) { _isAnimationComplete = true; return; }

	_buf0 = colors; _buf1 = colors; _buf2 = colors;
	_median = colors;
	_current = colors;
	_target  = std::move(colors);

	_isAnimationComplete = true;
	_lastUpdate = startTimeMs;
}

void InfiniteMedianEmaInterpolator::setTargetColors(std::vector<float3>&& new_rgb_targets,
                                                    long long startTimeMs, bool /*debug*/)
{
	if (new_rgb_targets.empty()) return;

	if (_buf0.size() != new_rgb_targets.size())
	{
		resetToColors(new_rgb_targets, startTimeMs);
		return;
	}

	_buf2 = std::move(_buf1);
	_buf1 = std::move(_buf0);
	_buf0 = std::move(new_rgb_targets);

	for (size_t i = 0; i < _median.size(); ++i)
	{
		for (int k = 0; k < 3; ++k)
		{
			const float a = _buf0[i][k], b = _buf1[i][k], c = _buf2[i][k];
			_median[i][k] = std::max(std::min(a, b), std::min(std::max(a, b), c));
		}
	}

	_target = _median;
	_isAnimationComplete = false;
}

void InfiniteMedianEmaInterpolator::updateCurrentColors(long long currentTimeMs, float /*minBrightness*/)
{
	if (_isAnimationComplete || _current.empty()) { _lastUpdate = currentTimeMs; return; }

	const float dt = std::clamp(static_cast<float>(currentTimeMs - _lastUpdate), 0.001f, 100.0f);
	_lastUpdate = currentTimeMs;

	const float alpha = 1.0f - std::pow(0.5f, dt / std::max(_halfLifeMs, 1.0f));

	_isAnimationComplete = true;

	for (size_t i = 0; i < _current.size(); ++i)
	{
		const float3 diff = _target[i] - _current[i];
		_current[i] += diff * alpha;

		if (linalg::maxelem(linalg::abs(diff)) > 0.0015f)
			_isAnimationComplete = false;
	}
}

SharedOutputColors InfiniteMedianEmaInterpolator::getCurrentColors(float minBrightness)
{
	if (minBrightness <= 0.0f)
		return std::make_shared<std::vector<float3>>(_current);

	auto out = _current;
	for (auto& c : out) c = linalg::clamp(c, minBrightness, 1.0f);
	return std::make_shared<std::vector<float3>>(std::move(out));
}

void InfiniteMedianEmaInterpolator::test()
{
	InfiniteMedianEmaInterpolator interp;
	interp.setTransitionDuration(150.0f);
	interp.resetToColors({ {0,0,0} }, 0);
	interp.setTargetColors({ {1,0,0} }, 0);

	std::cout << "Median+EMA test:\n";
	for (long long t = 0; t <= 500; t += 50)
	{
		interp.updateCurrentColors(t, 0.0f);
		auto c = interp.getCurrentColors();
		std::cout << std::setw(4) << t << "ms: "
		          << std::fixed << std::setprecision(3)
		          << (*c)[0].x << " " << (*c)[0].y << " " << (*c)[0].z << "\n";
	}
}