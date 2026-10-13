#pragma once

#ifndef PCH_ENABLED
	#include <QJsonDocument>
	#include <QMutex>
	#include <QVector>

	#include <vector>
#endif

#include <infinite-color-engine/InfiniteInterpolator.h>

/**
 * Steady-state Kalman (alpha-beta) filter per LED channel.
 * Constant-velocity model. stiffness -> process noise, damping -> meas. noise.
 */
class InfiniteKalmanInterpolator : public InfiniteInterpolator
{
public:
	InfiniteKalmanInterpolator();

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
	struct ChannelState
	{
		linalg::aliases::float3 pos{ 0, 0, 0 };
		linalg::aliases::float3 vel{ 0, 0, 0 };
	};

	std::vector<ChannelState> _state;
	std::vector<linalg::aliases::float3> _measurement;

	float _processNoise     = 0.02f;
	float _measurementNoise = 0.05f;
	long long _lastUpdate   = 0;
};