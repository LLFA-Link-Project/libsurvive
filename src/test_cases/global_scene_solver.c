#include "test_case.h"
#include "../survive_config.h"
#include "../survive_private.h"
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

static int solve_scene_below_previous_floor(SurviveObject *so, PoserData *pd) {
	PoserDataGlobalScenes *scenes = (PoserDataGlobalScenes *)pd;
	scenes->scenes[0].pose.Pos[2] -= .25;
	return 0;
}

TEST(GlobalScenes, CalibrationKeepsOutputFloorFixedUnlessRequested) {
	for (int enabled = 0; enabled < 2; enabled++) {
		struct SurviveContext_private private = {0};
		SurviveContext ctx = {.PoserFn = solve_scene_below_previous_floor, .objs_ct = 1, .floor_offset = .1,
			.private_members = &private};
		struct config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
		ctx.temporary_config_values = &temporary;
		ctx.global_config_values = &temporary;
		survive_configs(&ctx, "configfile", SC_SET | SC_OVERRIDE, "/dev/null");
		if (enabled)
			survive_configi(&ctx, "gss-auto-floor-height", SC_SET | SC_OVERRIDE, 1);
		SurviveObject live = {.ctx = &ctx};
		SurviveObject *objects[] = {&live};
		ctx.objs = objects;
		global_scene_solver gss = {.ctx = &ctx, .scenes_cnt = 1, .scenes_lock = OGCreateMutex()};
		gss.scenes[0].so = &live;
		gss.scenes[0].pose.Rot[0] = 1;
		global_scene_solver_attach_config(&ctx, &gss);
		for (int i = 0; i < 3; i++) {
			ASSERT_EQ(run_optimization(&gss), true);
			ASSERT_DOUBLE_EQ(survive_get_floor_offset(&ctx), enabled ? -.25 * (i + 1) : .1);
		}
		global_scene_solver_detach_config(&ctx, &gss);
		OGDeleteMutex(gss.scenes_lock);
		for (int i = 0; i < temporary.used_entries; i++) {
			ASSERT_EQ(temporary.config_entries[i].update_list, NULL);
			free(temporary.config_entries[i].tag);
			free(temporary.config_entries[i].data);
		}
		free(temporary.config_entries);
		OGDeleteMutex(temporary.write_lock);
	}
	return 0;
}

TEST(GlobalScenes, SparseCandidatePreservesAcceptedScene) {
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
	so.sensor_ct = 4;
	ASSERT_EQ(add_scenes(&gss, &so), 0);
	ASSERT_EQ(gss.scenes_cnt, GSS_NUM_STORED_SCENES);
	ASSERT_DOUBLE_EQ(gss.scenes[0].pose.Pos[0], 123.);
	ASSERT_EQ(gss.scenes[0].meas_cnt, 1);
	ASSERT_DOUBLE_EQ(gss.scenes[0].meas[0].value, .75);
	// Useful but insufficient data is another rejection, with the same guarantee.
	memset(gss.coverage, 0, sizeof(gss.coverage));
	ASSERT_EQ(add_scenes(&gss, &so), 0);
	ASSERT_DOUBLE_EQ(gss.scenes[0].pose.Pos[0], 123.);
	ASSERT_EQ(gss.scenes[0].meas_cnt, 1);
	ASSERT_DOUBLE_EQ(gss.scenes[0].meas[0].value, .75);
	so.sensor_ct = 6;

	// Complete redundant data refreshes the next full-ring slot without claiming
	// new coverage. Keeping old scenes indefinitely regressed long BS 2.0 replays.
	for (int axis = 0; axis < 2; axis++)
		for (int bin = 0; bin < NUM_BINS; bin++)
			gss.coverage[0][axis][bin] = gss.desired_coverage;
	ASSERT_EQ(add_scenes(&gss, &so), 0);
	ASSERT_EQ(gss.scenes_cnt, GSS_NUM_STORED_SCENES);
	ASSERT_DOUBLE_EQ(gss.scenes[0].pose.Pos[0], 0.);
	ASSERT_EQ(gss.scenes[0].meas_cnt, 12);
	ASSERT_DOUBLE_EQ(gss.scenes[0].meas[0].value, .1);

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

// The main close path owns poll_sema while a solver may need to retake it.
// No hardware or optimization is required to reproduce that lock cycle.
struct close_waiter {
	global_scene_solver *gss;
	og_sema_t ready;
};

static void *close_waiter_thread(void *userdata) {
	struct close_waiter *waiter = userdata;
	global_scene_solver *gss = waiter->gss;
	OGLockMutex(gss->data_available_lock);
	OGUnlockSema(waiter->ready);
	while (gss->active)
		OGWaitCond(gss->data_available, gss->data_available_lock);
	OGUnlockMutex(gss->data_available_lock);
	// Model the optimizer returning and reacquiring the context lock.
	survive_get_ctx_lock(gss->ctx);
	survive_release_ctx_lock(gss->ctx);
	return NULL;
}

static int test_close_solver(bool reacquire_context) {
	struct SurviveContext_private private = {.poll_sema = OGCreateSema()};
	config_entry settings[] = {
	    {.tag = "gss-threaded", .type = CONFIG_UINT32, .numeric.i = 1},
	    {.tag = "gss-desired-coverage", .type = CONFIG_UINT32, .numeric.i = 30},
	    {.tag = "gss-auto-floor-height", .type = CONFIG_UINT32, .numeric.i = 1},
	};
	config_group temporary = {.config_entries = settings, .used_entries = 3, .write_lock = OGCreateMutex()};
	SurviveContext ctx = {.private_members = &private, .temporary_config_values = &temporary};
	global_scene_solver *gss = calloc(1, sizeof(*gss));
	gss->ctx = &ctx;
	global_scene_solver_attach_config(&ctx, gss);
	gss->threaded = true;
	gss->active = true;
	gss->data_available = OGCreateConditionVariable();
	gss->data_available_lock = OGCreateMutex();
	gss->scenes_lock = OGCreateMutex();
	struct close_waiter waiter = {.gss = gss, .ready = OGCreateSema()};
	gss->thread = OGCreateThread(reacquire_context ? close_waiter_thread : survive_threaded_gss_thread_fn,
	                            "close waiter", reacquire_context ? (void *)&waiter : (void *)gss);
	if (reacquire_context)
		OGLockSema(waiter.ready);
	// poll_sema starts locked, matching survive_close's calling convention.
	ASSERT_EQ(DriverRegGlobalSceneSolverClose(&ctx, gss), 0);
	OGDeleteSema(waiter.ready);
	OGDeleteSema(private.poll_sema);
	OGDeleteMutex(temporary.write_lock);
	for (int i = 0; i < 3; i++)
		ASSERT_EQ(settings[i].update_list, NULL);
	return 0;
}

TEST(GlobalScenes, CloseReleasesContextWhileJoining) { return test_close_solver(true); }
TEST(GlobalScenes, CloseSleepingWorker) { return test_close_solver(false); }
