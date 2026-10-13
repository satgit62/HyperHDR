#pragma once

#ifndef PCH_ENABLED
	#include <QJsonDocument>
	#include <QMutex>
	#include <QVector>

	#include <vector>
#endif

#include <infinite-color-engine/InfiniteInterpolator.h>

/**
 * Adaptive EMA interpolator.
 *
 * Exponential moving average with an adaptive alpha: small changes are
 * heavily smoothed (noise rejection), large changes (scene cuts) jump
 * immediately. Great for noisy captures.
 */
class InfiniteAdaptiveEmaInterpolator : public InfiniteInterpolator
{
public:
	InfiniteAdaptiveEmaInterpolator();

	void setTargetColors(std::vector<linalg::aliases::float3>&& new_rgb_targets,
	                     long long startTimeMs, bool debug = false) override;
	void updateCurrentColors(long long currentTimeMs, float minBrightness) override;
	SharedOutputColors getCurrentColors(float minBrightness = 0.f) override;
	void resetState() override;

	void setTransitionDuration(float durationMs) override;
	void setSmoothingFactor(float factor) override;
	void setSpringiness(float stiffness, float damping) override;

	void resetToColors(std::vector<linalg::aliases::float3> colors, long long startTimeMs);
	static void test();

private:
	std::vector<linalg::aliases::float3> _current;
	std::vector<linalg::aliases::float3> _target;

	float _halfLifeMs   = 60.0f;   // ms to reach half of the residual
	float _triggerDelta = 0.15f;   // magnitude above which we consider a scene cut
	long long _lastUpdate = 0;
};