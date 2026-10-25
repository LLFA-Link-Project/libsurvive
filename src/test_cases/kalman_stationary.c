#include "../survive_config.h"
#include "../survive_kalman_tracker.h"
#include "test_case.h"
#include <stddef.h>

// Exercise the actual joint Kalman update, with a position/velocity covariance
// and a small velocity drift that a handheld detector can classify as still.
static int stationary_update(FLT ramp, FLT stationary_seconds, bool no_light,
                             bool error_state, FLT *position, FLT *velocity, bool recent_bad_pose) {
	SurviveContext ctx = {0};
	config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
	ctx.temporary_config_values = &temporary;
	survive_configi(&ctx, "kalman-minimize-state-space", SC_SET | SC_OVERRIDE, 0);
	survive_configi(&ctx, "kalman-use-error-space", SC_SET | SC_OVERRIDE, error_state);
	survive_configf(&ctx, "kalman-zvu-stationary-ramp", SC_SET | SC_OVERRIDE, ramp);
	SurviveObject so = {.ctx = &ctx, .timebase_hz = 48000000, .imu_freq = 250};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	tracker.model.t = tracker.imu_bias_model.t = 1;
	tracker.stats.obs_count = 16;
	tracker.last_unreliable_pose_time = recent_bad_pose ? .95 : 0;
	tracker.acc_var = tracker.gyro_var = -1; // isolate the stationary observation
	tracker.last_light_time = no_light ? 0 : 1;
	tracker.state.Velocity.Pos[0] = .05;
	cn_set_zero(&tracker.model.P);
	for (int i = 0; i < tracker.model.P.rows; i++)
		cnMatrixSet(&tracker.model.P, i, i, 1e-4);
	int v = error_state ? offsetof(SurviveKalmanErrorModel, Velocity.Pos) / sizeof(FLT)
	                    : offsetof(SurviveKalmanModel, Velocity.Pos) / sizeof(FLT);
	cnMatrixSet(&tracker.model.P, v, v, 5e-5);
	cnMatrixSet(&tracker.model.P, 0, v, 3e-6);
	cnMatrixSet(&tracker.model.P, v, 0, 3e-6);
	so.activations.last_light = so.activations.last_imu = 48000000;
	so.activations.last_movement = 48000000 - (uint64_t)(stationary_seconds * 48000000);
	PoserDataIMU imu = {.hdr = {.pt = POSERDATA_IMU, .timecode = 48000000}, .accel = {0, 0, 1}};
	survive_kalman_tracker_integrate_imu(&tracker, &imu);
	*position = tracker.state.Pose.Pos[0];
	*velocity = tracker.state.Velocity.Pos[0];
	ASSERT_EQ(cn_is_finite(&tracker.model.P), true);
	for (int i = 0; i < tracker.model.P.rows; i++)
		ASSERT_EQ((cnMatrixGet(&tracker.model.P, i, i) >= 0), true);
	survive_kalman_tracker_free(&tracker);
	for (int i = 0; i < temporary.used_entries; i++) {
		ASSERT_EQ(temporary.config_entries[i].update_list, NULL);
		free(temporary.config_entries[i].tag);
		free(temporary.config_entries[i].data);
	}
	free(temporary.config_entries);
	OGDeleteMutex(temporary.write_lock);
	return 0;
}

TEST(KalmanStationary, SoftOnsetAvoidsPositionSnap) {
	for (int error_state = 0; error_state <= 1; error_state++) {
		FLT hard_p, hard_v, soft_p, soft_v;
		ASSERT_EQ(stationary_update(0, .104, false, error_state, &hard_p, &hard_v, false), 0);
		ASSERT_EQ(stationary_update(.1, .104, false, error_state, &soft_p, &soft_v, false), 0);
		ASSERT_EQ((fabs(hard_p) > .002), true);
		ASSERT_EQ((fabs(soft_p) < .0001), true);
		ASSERT_EQ((soft_v > hard_v && soft_v < .05), true);
	}
	return 0;
}

TEST(KalmanStationary, SustainedRestReachesFullConstraint) {
	FLT hard_p, hard_v, soft_p, soft_v;
	ASSERT_EQ(stationary_update(0, .4, false, true, &hard_p, &hard_v, false), 0);
	ASSERT_EQ(stationary_update(.1, .4, false, true, &soft_p, &soft_v, false), 0);
	ASSERT_DOUBLE_EQ(hard_p, soft_p);
	ASSERT_DOUBLE_EQ(hard_v, soft_v);
	return 0;
}

TEST(KalmanStationary, MovingAndNoLightBehaviorIsPreserved) {
	for (int no_light = 0; no_light <= 1; no_light++) {
		FLT hard_p, hard_v, soft_p, soft_v;
		ASSERT_EQ(stationary_update(0, .05, no_light, true, &hard_p, &hard_v, false), 0);
		ASSERT_EQ(stationary_update(.1, .05, no_light, true, &soft_p, &soft_v, false), 0);
		ASSERT_DOUBLE_EQ(hard_p, soft_p);
		ASSERT_DOUBLE_EQ(hard_v, soft_v);
	}
	return 0;
}

struct micro_motion_result { FLT position_rms, angle_rms, position_gain, angle_gain; };

static int micro_motion(FLT ramp, bool rotate, FLT velocity_sigma, struct micro_motion_result *result) {
	SurviveContext ctx = {0};
	config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
	ctx.temporary_config_values = &temporary;
	survive_configi(&ctx, "kalman-minimize-state-space", SC_SET | SC_OVERRIDE, 0);
	survive_configf(&ctx, "kalman-zvu-stationary-ramp", SC_SET | SC_OVERRIDE, ramp);
	survive_configf(&ctx, "kalman-zvu-stationary-gyro-scale", SC_SET | SC_OVERRIDE, ramp > 0 ? .02 : 0);
	survive_configf(&ctx, "kalman-zvu-stationary-velocity-sigma", SC_SET | SC_OVERRIDE, velocity_sigma);
	SurviveObject so = {.ctx = &ctx, .timebase_hz = 48000000, .imu_freq = 250};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	so.tracker = &tracker;
	FLT p_error = 0, r_error = 0, p_sin = 0, p_cos = 0, r_sin = 0, r_cos = 0;
	int samples = 0;
	for (int i = 1; i <= 5000; i++) {
		FLT t = i * .004, phase = LINMATHPI * t;
		FLT x = .002 * sin(phase), angle = rotate ? .02 * sin(phase) : 0;
		FLT acceleration = -.002 * LINMATHPI * LINMATHPI * sin(phase);
		survive_long_timecode tick = 48000000 + i * UINT64_C(192000);
		so.activations.last_light = so.activations.last_imu = tick;
		// Short apparent pauses are common while holding a tracker. The
		// ground truth here keeps moving smoothly through those detections.
		so.activations.last_movement = tick - (i % 75) * UINT64_C(192000);
		PoserDataIMU imu = {.hdr = {.pt = POSERDATA_IMU, .timecode = tick},
			.accel = {cos(angle) * acceleration / 9.80665, -sin(angle) * acceleration / 9.80665, 1},
			.gyro = {0, 0, rotate ? .02 * LINMATHPI * cos(phase) : 0}};
		if (i % 5 == 0) {
			SurvivePose truth = {.Pos = {x, 0, 0}, .Rot = {cos(angle / 2), 0, 0, sin(angle / 2)}};
			PoserData pd = {.pt = POSERDATA_LIGHT_GEN2, .timecode = tick};
			survive_kalman_tracker_integrate_observation(&pd, &tracker, &truth, 0);
		}
		survive_kalman_tracker_integrate_imu(&tracker, &imu);
		if (t > 4) {
			FLT px = tracker.state.Pose.Pos[0];
			FLT a = 2 * atan2(tracker.state.Pose.Rot[3], tracker.state.Pose.Rot[0]);
			p_error += (px - x) * (px - x);
			r_error += (a - angle) * (a - angle);
			p_sin += px * sin(phase); p_cos += px * cos(phase);
			r_sin += a * sin(phase); r_cos += a * cos(phase);
			samples++;
		}
	}
	*result = (struct micro_motion_result){sqrt(p_error / samples), sqrt(r_error / samples),
		2 * hypot(p_sin, p_cos) / samples / .002, 2 * hypot(r_sin, r_cos) / samples / .02};
	survive_kalman_tracker_free(&tracker);
	for (int i = 0; i < temporary.used_entries; i++) {
		ASSERT_EQ(temporary.config_entries[i].update_list, NULL);
		free(temporary.config_entries[i].tag);
		free(temporary.config_entries[i].data);
	}
	free(temporary.config_entries);
	OGDeleteMutex(temporary.write_lock);
	return 0;
}

TEST(KalmanStationary, MicroMotionSurvivesIntermittentStationaryDetections) {
	struct micro_motion_result hard, soft;
	ASSERT_EQ(micro_motion(0, true, .1, &hard), 0);
	ASSERT_EQ(micro_motion(.5, true, .1, &soft), 0);
	fprintf(stderr, "Micro-motion hard/soft: position RMS %.6f/%.6f mm, angle RMS %.6f/%.6f deg, gains %.6f/%.6f %.6f/%.6f\n",
	        hard.position_rms * 1000, soft.position_rms * 1000, hard.angle_rms * 180 / LINMATHPI,
	        soft.angle_rms * 180 / LINMATHPI, hard.position_gain, soft.position_gain, hard.angle_gain, soft.angle_gain);
	ASSERT_EQ((fabs(soft.position_gain - 1) < .05), true);
	ASSERT_EQ((fabs(soft.angle_gain - 1) < .05), true);
	ASSERT_EQ((soft.position_rms < .0001), true);
	ASSERT_EQ((soft.angle_rms < .001), true);
	ASSERT_EQ((soft.position_rms < hard.position_rms), true);
	ASSERT_EQ((soft.angle_rms < hard.angle_rms), true);
	return 0;
}

TEST(KalmanStationary, SmallTranslationDoesNotRequireRotation) {
	struct micro_motion_result hard, soft;
	ASSERT_EQ(micro_motion(0, false, .1, &hard), 0);
	ASSERT_EQ(micro_motion(.5, false, .1, &soft), 0);
	fprintf(stderr, "Translation hard/soft RMS %.6f/%.6f mm, gains %.6f/%.6f\n",
	        hard.position_rms * 1000, soft.position_rms * 1000, hard.position_gain, soft.position_gain);
	ASSERT_EQ((fabs(soft.position_gain - 1) < .05), true);
	ASSERT_EQ((soft.position_rms < .0001), true);
	ASSERT_EQ((soft.position_rms < hard.position_rms), true);
	return 0;
}

TEST(KalmanStationary, OpticalRecoveryRetainsOriginalConstraint) {
	FLT hard_p, hard_v, soft_p, soft_v;
	ASSERT_EQ(stationary_update(0, .104, false, true, &hard_p, &hard_v, true), 0);
	ASSERT_EQ(stationary_update(.5, .104, false, true, &soft_p, &soft_v, true), 0);
	ASSERT_DOUBLE_EQ(hard_p, soft_p);
	ASSERT_DOUBLE_EQ(hard_v, soft_v);
	return 0;
}
