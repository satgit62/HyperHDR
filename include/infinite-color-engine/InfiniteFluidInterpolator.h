/* InfiniteFluidInterpolator.h
 *
 *  MIT License
 *  Copyright (c) 2020-2026 awawa-dev
 *
 *  Fluid interpolator: strong spatial blur + adaptive Oklab EMA.
 *  Provides smooth, continuous color transitions along the LED strip.
 */

#pragma once

#include <vector>
#include <linalg.h>
#include <infinite-color-engine/InfiniteInterpolator.h>

class InfiniteFluidInterpolator : public InfiniteInterpolator
{
public:
	InfiniteFluidInterpolator();
	virtual ~InfiniteFluidInterpolator() = default;

	// Overridden methods from InfiniteInterpolator
	void setTransitionDuration(float durationMs) override;
	void setSpringiness(float stiffness, float damping) override;
	void setSmoothingFactor(float factor) override;
	void setMaxLuminanceChangePerFrame(float maxChangePerStep) override;
	
	// Specific to this interpolator
	void setSpatialStrength(float strength); // Range: 0.0 to 0.45

	void resetState() override;
	
	// Not virtual in base class, so no 'override' here
	void resetToColors(std::vector<linalg::aliases::float3> colors, long long startTimeMs);
	
	void setTargetColors(std::vector<linalg::aliases::float3>&& new_rgb_targets, long long startTimeMs, bool debug = false) override;
	void updateCurrentColors(long long currentTimeMs, float minBrightness) override;
	SharedOutputColors getCurrentColors(float minBrightness) override;

private:
	std::vector<linalg::aliases::float3> _currentOklab;
	std::vector<linalg::aliases::float3> _targetOklab;
	std::vector<linalg::aliases::float3> _outputRgb;

	float _halfLifeMs = 100.0f;
	float _spatialStrength = 0.40f;
	long long _lastUpdate = 0;
	// _isAnimationComplete is inherited from InfiniteInterpolator (protected)
};