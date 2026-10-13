#pragma once

#ifndef PCH_ENABLED
	#include <QJsonDocument>
	#include <QMutex>
	#include <QVector>

	#include <vector>
#endif

#include <infinite-color-engine/InfiniteInterpolator.h>

/**
 * Median-of-3 + EMA. Single-frame spikes (HDMI glitches, VRR changes,
 * subtitle flashes) are removed by the median filter before the EMA smooths.
 */
class InfiniteMedianEmaInterpolator : public InfiniteInterpolator
{
public:
	InfiniteMedianEmaInterpolator();

	void setTargetColors(std::vector<linalg::aliases::float3>&& new_rgb_targets,
	                     long long startTimeMs, bool debug = false) override;
	void updateCurrentColors(long long currentTimeMs, float minBrightness) override;
	SharedOutputColors getCurrentColors(float minBrightness = 0.f) override;
	void resetState() override;

	void setTransitionDuration(float durationMs) override;
	void setSmoothingFactor(float factor) override;

	void resetToColors(std::vector<linalg::aliases::float3> colors, long long startTimeMs);
	static void test();

private:
	std::vector<linalg::aliases::float3> _buf0, _buf1, _buf2;
	std::vector<linalg::aliases::float3> _median;
	std::vector<linalg::aliases::float3> _current;
	std::vector<linalg::aliases::float3> _target;

	float _halfLifeMs = 70.0f;
	long long _lastUpdate = 0;
};