#include "test_case.h"
#include "../driver_global_scene_solver.c"

TEST(GlobalScenes, RejectedCandidatePreservesAcceptedScene) {
	SurviveContext ctx = {.activeLighthouses = 1};
	SurviveObject so = {.ctx = &ctx, .sensor_ct = 6, .OutPoseIMU = {.Rot = {1}}};
	so.activations.so = &so;
	so.activations.lh_gen = 1;
	so.activations.last_light = 100000;
	so.activations.last_movement = 1;
	for (int sensor = 0; sensor < so.sensor_ct; sensor++) {
		for (int axis = 0; axis < 2; axis++) {
			so.activations.angles[sensor][0][axis] = .1;
			so.activations.timecode[sensor][0][axis] = 100000;
		}
	}
	global_scene_solver gss = {.ctx = &ctx, .scenes_cnt = GSS_NUM_STORED_SCENES, .desired_coverage = 30};
	for (int i = 0; i < GSS_NUM_STORED_SCENES; i++) {
		gss.scenes[i].so = &so;
		gss.scenes[i].pose.Pos[0] = 123;
		gss.scenes[i].meas_cnt = 1;
		gss.scenes[i].meas = calloc(1, sizeof(*gss.scenes[i].meas));
		gss.scenes[i].meas[0].value = .75;
	}
	for (int axis = 0; axis < 2; axis++)
		for (int bin = 0; bin < NUM_BINS; bin++)
			gss.coverage[0][axis][bin] = gss.desired_coverage;
	ASSERT_EQ(add_scenes(&gss, &so), 0);
	ASSERT_EQ(gss.scenes_cnt, GSS_NUM_STORED_SCENES);
	ASSERT_DOUBLE_EQ(gss.scenes[0].pose.Pos[0], 123.);
	ASSERT_EQ(gss.scenes[0].meas_cnt, 1);
	ASSERT_DOUBLE_EQ(gss.scenes[0].meas[0].value, .75);
	// Useful but insufficient data is another rejection, with the same guarantee.
	memset(gss.coverage, 0, sizeof(gss.coverage));
	so.sensor_ct = 4;
	ASSERT_EQ(add_scenes(&gss, &so), 0);
	ASSERT_DOUBLE_EQ(gss.scenes[0].pose.Pos[0], 123.);
	ASSERT_EQ(gss.scenes[0].meas_cnt, 1);
	ASSERT_DOUBLE_EQ(gss.scenes[0].meas[0].value, .75);
	so.sensor_ct = 6;

	// An accepted candidate must replace the next ring slot and own its measurements.
	memset(gss.coverage, 0, sizeof(gss.coverage));
	ASSERT_EQ(add_scenes(&gss, &so), 1);
	ASSERT_EQ(gss.scenes_cnt, GSS_NUM_STORED_SCENES + 1);
	ASSERT_DOUBLE_EQ(gss.scenes[0].pose.Pos[0], 0.);
	ASSERT_EQ(gss.scenes[0].meas_cnt, 12);
	ASSERT_DOUBLE_EQ(gss.scenes[0].meas[0].value, .1);
	for (int i = 0; i < GSS_NUM_STORED_SCENES; i++)
		free(gss.scenes[i].meas);
	return 0;
}
