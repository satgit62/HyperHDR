#pragma once

#ifndef PCH_ENABLED
	#include <QJsonDocument>
	#include <QMutex>
	#include <QVector>

	#include <vector>
#endif

#include <infinite-color-engine/InfiniteInterpolator.h>

/**
 * Oklab perceptual interpolator with optional spatial smoothing.
 *
 * Interpolates in Oklab (Björn Ottosson, 2020), a perceptually uniform
 * color space. This removes the "muddy" mid-tones that appear when
 * interpolating red->green or cyan->yellow in RGB or YUV.
 *
 * Optionally applies a 3-tap spatial smoothing across the LED strip
 * (in Oklab) so neighbours blend more gracefully.
 */
class InfinitePerceptualInterpolator : public InfiniteInterpolator
{
public:
	InfinitePerceptualInterpolator();

	void setTargetColors(std::vector<linalg::aliases::float3>&& new_rgb_targets,
	                     long long startTimeMs, bool debug = false) override;
	void updateCurrentColors(long long currentTimeMs, float minBrightness) override;
	SharedOutputColors getCurrentColors(float minBrightness = 0.f) override;
	void resetState() override;

	void setTransitionDuration(float durationMs) override;
	void setSpringiness(float stiffness, float damping) override;
	void setSmoothingFactor(float factor) override;

	// Spatial smoothing strength along the strip (0.0 = off, ~0.45 = strong)
	void setSpatialSmoothing(float strength);

	void resetToColors(std::vector<linalg::aliases::float3> colors, long long startTimeMs);
	static void test();

private:
	// Internal state in Oklab (L, a, b)
	std::vector<linalg::aliases::float3> _targetColorsOklab;
	std::vector<linalg::aliases::float3> _currentColorsOklab;
	std::vector<linalg::aliases::float3> _velocitiesOklab;

	// Cached output in nonlinear sRGB
	std::vector<linalg::aliases::float3> _currentColorsRGB;

	float _initialDuration = 150.0f;
	long long _startAnimationTimeMs = 0;
	long long _targetTime = 0;
	long long _lastUpdate = 0;
	float _stiffness = 150.0f;
	float _damping = 26.0f;
	float _smoothingFactor = 0.0f;
	float _spatialSmoothing = 0.0f;
};