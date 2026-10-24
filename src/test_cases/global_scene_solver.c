#include "test_case.h"
#include "../driver_global_scene_solver.c"

static int deferred_solve_calls;
static int deferred_solve(SurviveObject *so, PoserData *pd) {
	PoserDataGlobalScenes *scenes = (PoserDataGlobalScenes *)pd;
	ASSERT_EQ(so, so->ctx->objs[0]);
	ASSERT_EQ(scenes->scenes_cnt, 1);
	ASSERT_EQ(scenes->scenes[0].so, so);
	ASSERT_DOUBLE_EQ(scenes->scenes[0].meas[0].value, .25);
	deferred_solve_calls++;
	return 0;
}

TEST(GlobalScenes, DeferredSolveDiscardsDisconnectedObjects) {
	SurviveContext ctx = {.PoserFn = deferred_solve, .objs_ct = 1};
	SurviveObject live = {.ctx = &ctx};
	SurviveObject *objects[] = {&live};
	ctx.objs = objects;
	SurviveObject *removed = calloc(1, sizeof(*removed));
	global_scene_solver gss = {.ctx = &ctx, .scenes_cnt = 3, .scenes_lock = OGCreateMutex()};
	for (int i = 0; i < 3; i++) {
		gss.scenes[i].so = i == 1 ? &live : removed;
		gss.scenes[i].meas_cnt = 1;
		gss.scenes[i].meas = calloc(1, sizeof(*gss.scenes[i].meas));
		gss.scenes[i].meas[0].value = i == 1 ? .25 : .75;
	}
	free(removed);
	deferred_solve_calls = 0;
	ASSERT_EQ(run_optimization(&gss), true);
	ASSERT_EQ(deferred_solve_calls, 1);
	ASSERT_EQ(gss.scenes_cnt, 1);

	// Losing the last object must not invoke the poser with an empty object list.
	ctx.objs_ct = 0;
	ctx.objs = NULL;
	ASSERT_EQ(run_optimization(&gss), false);
	ASSERT_EQ(gss.scenes_cnt, 0);
	ASSERT_EQ(deferred_solve_calls, 1);
	OGDeleteMutex(gss.scenes_lock);
	for (int i = 0; i < 3; i++)
		free(gss.scenes[i].meas);
	return 0;
}

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
