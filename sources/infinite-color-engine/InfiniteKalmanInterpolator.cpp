/* InfiniteKalmanInterpolator.cpp
 *
 *  MIT License
 *  Copyright (c) 2020-2026 awawa-dev
 *
 *  Steady-state Kalman (alpha-beta) filter per LED channel.
 */

#ifndef PCH_ENABLED
	#include <cmath>
	#include <algorithm>
	#include <vector>
	#include <iostream>
	#include <iomanip>
#endif

#include <infinite-color-engine/InfiniteKalmanInterpolator.h>

using namespace linalg::aliases;

InfiniteKalmanInterpolator::InfiniteKalmanInterpolator() = default;

void InfiniteKalmanInterpolator::setTransitionDuration(float durationMs)
{
	_processNoise = std::clamp(1.25f / std::max(durationMs, 1.0f), 0.002f, 0.10f);
}

void InfiniteKalmanInterpolator::setSpringiness(float stiffness, float damping)
{
	if (stiffness > 0.0f)
		_processNoise     = std::clamp(stiffness / 5000.0f, 0.002f, 0.10f);
	if (damping > 0.0f)
		_measurementNoise = std::clamp(damping / 500.0f, 0.01f, 0.5f);
}

void InfiniteKalmanInterpolator::resetState()
{
	_isAnimationComplete = true;
	_lastUpdate = 0;
	_state.clear();
	_measurement.clear();
}

void InfiniteKalmanInterpolator::resetToColors(std::vector<float3> colors, long long startTimeMs)
{
	if (colors.empty()) { _isAnimationComplete = true; return; }

	_state.clear();
	_state.reserve(colors.size());
	for (const auto& c : colors)
		_state.push_back({ c, float3{ 0, 0, 0 } });

	_measurement = std::move(colors);
	_isAnimationComplete = true;
	_lastUpdate = startTimeMs;
}

void InfiniteKalmanInterpolator::setTargetColors(std::vector<float3>&& new_rgb_targets,
                                                 long long startTimeMs, bool /*debug*/)
{
	if (new_rgb_targets.empty()) return;

	if (_state.size() != new_rgb_targets.size())
	{
		resetToColors(new_rgb_targets, startTimeMs);
		return;
	}
	_measurement = std::move(new_rgb_targets);
	_isAnimationComplete = false;
}

void InfiniteKalmanInterpolator::updateCurrentColors(long long currentTimeMs, float /*minBrightness*/)
{
	if (_isAnimationComplete || _state.empty())
	{
		_lastUpdate = currentTimeMs;
		return;
	}

	const float dt = std::clamp(static_cast<float>(currentTimeMs - _lastUpdate), 0.001f, 100.0f);
	_lastUpdate = currentTimeMs;

	const float q = _processNoise * _processNoise;
	const float r = _measurementNoise * _measurementNoise;

	const float qdt2 = q * dt * dt;
	const float sq   = std::sqrt(q * r);
	const float lambda = qdt2 + 4.0f * sq * dt + 4.0f * r;
	const float alpha  = (qdt2 + 2.0f * sq * dt) / std::max(lambda, 1e-9f);
	const float beta   = (2.0f * q * dt)          / std::max(lambda, 1e-9f);

	_isAnimationComplete = true;

	for (size_t i = 0; i < _state.size(); ++i)
	{
		auto& s = _state[i];
		const float3 predPos = s.pos + s.vel * (dt * 0.001f);
		const float3 residual = _measurement[i] - predPos;

		s.pos = predPos + residual * alpha;
		s.vel = s.vel + residual * (beta / (dt * 0.001f));
		s.pos = linalg::clamp(s.pos, 0.0f, 1.0f);

		if (linalg::maxelem(linalg::abs(residual)) > 0.0015f)
			_isAnimationComplete = false;
	}
}

SharedOutputColors InfiniteKalmanInterpolator::getCurrentColors(float minBrightness)
{
	std::vector<float3> out(_state.size());
	for (size_t i = 0; i < _state.size(); ++i)
		out[i] = linalg::clamp(_state[i].pos, minBrightness, 1.0f);
	return std::make_shared<std::vector<float3>>(std::move(out));
}

void InfiniteKalmanInterpolator::test()
{
	InfiniteKalmanInterpolator interp;
	interp.setTransitionDuration(150.0f);
	interp.resetToColors({ {0,0,0} }, 0);
	interp.setTargetColors({ {1,0,0} }, 0);

	std::cout << "Kalman test:\n";
	for (long long t = 0; t <= 500; t += 50)
	{
		interp.updateCurrentColors(t, 0.0f);
		auto c = interp.getCurrentColors();
		std::cout << std::setw(4) << t << "ms: "
		          << std::fixed << std::setprecision(3)
		          << (*c)[0].x << " " << (*c)[0].y << " " << (*c)[0].z << "\n";
	}
}