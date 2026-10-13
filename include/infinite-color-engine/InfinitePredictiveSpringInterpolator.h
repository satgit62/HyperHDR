#pragma once

#ifndef PCH_ENABLED
	#include <QJsonDocument>
	#include <QMutex>
	#include <QVector>

	#include <vector>
#endif

#include <infinite-color-engine/InfiniteInterpolator.h>

/**
 * Predictive spring with lookahead. Estimates the target's velocity and
 * predicts where it will be in a few ms, removing the lag that plain
 * springs show while keeping smoothness (no overshoot).
 */
class InfinitePredictiveSpringInterpolator : public InfiniteInterpolator
{
public:
	InfinitePredictiveSpringInterpolator();

	void setTargetColors(std::vector<linalg::aliases::float3>&& new_rgb_targets,
	                     long long startTimeMs, bool debug = false) override;
	void updateCurrentColors(long long currentTimeMs, float minBrightness) override;
	SharedOutputColors getCurrentColors(float minBrightness = 0.f) override;
	void resetState() override;

	void setTransitionDuration(float durationMs) override;
	void setSpringiness(float stiffness, float damping) override;

	void resetToColors(std::vector<linalg::aliases::float3> colors, long long startTimeMs);
	static void test();

private:
	std::vector<linalg::aliases::float3> _current;
	std::vector<linalg::aliases::float3> _target;
	std::vector<linalg::aliases::float3> _vel;
	std::vector<linalg::aliases::float3> _targetVel;

	float _stiffness    = 180.0f;
	float _damping      = 26.0f;
	float _lookaheadSec = 0.04f;
	long long _lastUpdate  = 0;
	long long _lastTargetT = 0;
};