#pragma once

#ifndef PCH_ENABLED
	#include <QJsonDocument>
	#include <QMutex>
	#include <QVector>

	#include <vector>
#endif

#include <infinite-color-engine/InfiniteInterpolator.h>

/**
 * Luma/Chroma split in Oklab: L is fast, chroma is slow.
 * Mimics human vision - no color smear on motion, sharp brightness.
 */
class InfiniteLumaChromaInterpolator : public InfiniteInterpolator
{
public:
	InfiniteLumaChromaInterpolator();

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
	std::vector<float>                            _L,   _Lt;
	std::vector<linalg::aliases::float2>          _ab,  _abt;

	float _lumaHalfLifeMs   = 40.0f;
	float _chromaHalfLifeMs = 150.0f;
	long long _lastUpdate   = 0;
};