#pragma once

#ifndef PCH_ENABLED
	#include <QJsonDocument>
	#include <QMutex>
	#include <QVector>

	#include <vector>
#endif

#include <infinite-color-engine/InfiniteInterpolator.h>

/**
 * Kinetic interpolator: aims for the "alive" look of commercial
 * Ambilight demos on real content.
 *
 * Combines five techniques into a single pipeline:
 *
 *   1. Oklab color space (perceptually uniform, no muddy mid-tones)
 *   2. Per-LED magnitude-adaptive attack (small change = slow, big = fast)
 *   3. Global scene-change detection (all LEDs snap on hard cuts)
 *   4. Content-aware spatial smoothing (less when the frame is moving)
 *   5. Chroma boost (compensates for NV12 4:2:0 desaturation)
 *
 * Shadow lift / minimum backlight is intentionally NOT included: the
 * InfiniteProcessing pipeline already provides that via the
 * "Minimum backlight level" setting (applyMinimalBacklight).
 *
 * Operates on the LED array (e.g. 272 elements) per frame, so the CPU
 * cost is negligible compared to the pixel-level mapping stage.
 */
class InfiniteKineticInterpolator : public InfiniteInterpolator
{
public:
	InfiniteKineticInterpolator();

	void setTargetColors(std::vector<linalg::aliases::float3>&& new_rgb_targets,
	                     long long startTimeMs, bool debug = false) override;
	void updateCurrentColors(long long currentTimeMs, float minBrightness) override;
	SharedOutputColors getCurrentColors(float minBrightness = 0.f) override;
	void resetState() override;

	void setTransitionDuration(float durationMs) override;
	void setSpringiness(float stiffness, float damping) override;
	void setSmoothingFactor(float factor) override;

	void setChromaBoost(float boost);
	void setSpatialStrength(float strength);

	void resetToColors(std::vector<linalg::aliases::float3> colors, long long startTimeMs);
	static void test();

private:
	std::vector<linalg::aliases::float3> _targetOklab;
	std::vector<linalg::aliases::float3> _currentOklab;
	std::vector<linalg::aliases::float3> _prevTargetOklab;

	std::vector<linalg::aliases::float3> _outputRgb;

	float _halfLifeMs       = 80.0f;
	float _sceneSensitivity = 0.5f;
	float _chromaBoost      = 0.20f;
	float _spatialStrength  = 0.20f;

	float _sceneChangeSmoothed = 0.0f;

	long long _lastUpdate = 0;
};