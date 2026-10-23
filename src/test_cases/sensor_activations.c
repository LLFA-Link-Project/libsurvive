#include "test_case.h"

TEST(SensorActivations, PairAfterDeviceClockWrap) {
	SurviveSensorActivations activations = {0};
	activations.last_light = (UINT64_C(1) << 32) + 1000;
	activations.lengths[0][0][0] = activations.lengths[0][0][1] = 100;
	activations.timecode[0][0][0] = activations.last_light - 100;
	activations.timecode[0][0][1] = activations.last_light - 50;
	ASSERT_EQ(SurviveSensorActivations_isPairValid(&activations, 150, 1000, 0, 0), true);
	ASSERT_EQ(SurviveSensorActivations_isPairValid(&activations, 49, 1000, 0, 0), false);
	ASSERT_EQ(SurviveSensorActivations_isPairValid(&activations, 150, 2000, 0, 0), false);
	return 0;
}

TEST(SensorActivations, RuntimeClockCanPrecedeDeviceClock) {
	SurviveSensorActivations activations = {0};
	SurviveSensorActivations_register_runtime(&activations, 48000000, 1000);
	ASSERT_DOUBLE_EQ(activations.runtime_offset, -999000);
	ASSERT_EQ(SurviveSensorActivations_runtime(&activations, 48000000), 1000);
	ASSERT_EQ(SurviveSensorActivations_runtime(&activations, 48048000), 2000);
	return 0;
}

TEST(SensorActivations, StaleReadingsDoNotBecomeRecentAfterClockWrap) {
	SurviveContext ctx = {.activeLighthouses = 1};
	ctx.bsd[0].PositionSet = true;
	SurviveObject so = {.ctx = &ctx, .sensor_ct = 1};
	SurviveSensorActivations activations = {.so = &so, .last_light = (UINT64_C(1) << 32) + 1000};
	for (int axis = 0; axis < 2; axis++) {
		activations.lengths[0][0][axis] = 100;
		activations.timecode[0][0][axis] = 999;
	}
	uint32_t count = 0, lighthouses = 0, axes = 0;
	size_t per_axis[NUM_GEN2_LIGHTHOUSES * 2] = {0};
	SurviveSensorActivations_valid_counts(&activations, 100, &count, &lighthouses, &axes, per_axis);
	ASSERT_EQ(count, 0);
	ASSERT_EQ(lighthouses, 0);
	ASSERT_EQ(axes, 0);
	ASSERT_EQ(per_axis[0], 0);
	ASSERT_EQ(per_axis[1], 0);
	SurviveSensorActivations_valid_counts(&activations, (UINT64_C(1) << 32) + 100,
										&count, &lighthouses, &axes, per_axis);
	ASSERT_EQ(count, 2);
	ASSERT_EQ(lighthouses, 1);
	ASSERT_EQ(per_axis[0], 1);
	ASSERT_EQ(per_axis[1], 1);
	return 0;
}
