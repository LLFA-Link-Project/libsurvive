#include "../survive_config.h"
#include "../survive_kalman_tracker.h"
#include "test_case.h"
#include <stddef.h>

// Exercise the actual joint Kalman update, with a position/velocity covariance
// and a small velocity drift that a handheld detector can classify as still.
static int stationary_update_with_options(FLT ramp, FLT stationary_seconds, bool no_light,
                             bool error_state, FLT *position, FLT *velocity, bool recent_bad_pose, bool controller,
                             int all_devices) {
	SurviveContext ctx = {0};
	config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
	ctx.temporary_config_values = &temporary;
	survive_configi(&ctx, "kalman-minimize-state-space", SC_SET | SC_OVERRIDE, 0);
	survive_configi(&ctx, "kalman-use-error-space", SC_SET | SC_OVERRIDE, error_state);
	survive_configf(&ctx, "kalman-zvu-stationary-ramp", SC_SET | SC_OVERRIDE, ramp);
	if (all_devices >= 0)
		survive_configi(&ctx, "kalman-zvu-motion-all-devices", SC_SET | SC_OVERRIDE, all_devices);
	SurviveObject so = {.ctx = &ctx, .object_type = SURVIVE_OBJECT_TYPE_OTHER, .timebase_hz = 48000000, .imu_freq = 250};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	tracker.model.t = tracker.imu_bias_model.t = 1;
	so.object_type = controller ? SURVIVE_OBJECT_TYPE_CONTROLLER : SURVIVE_OBJECT_TYPE_OTHER;
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

static int stationary_update(FLT ramp, FLT stationary_seconds, bool no_light,
                             bool error_state, FLT *position, FLT *velocity, bool recent_bad_pose, bool controller) {
	return stationary_update_with_options(ramp, stationary_seconds, no_light, error_state,
		position, velocity, recent_bad_pose, controller, -1);
}

TEST(KalmanStationary, SoftOnsetAvoidsPositionSnap) {
	for (int error_state = 0; error_state <= 1; error_state++) {
		FLT hard_p, hard_v, soft_p, soft_v;
		ASSERT_EQ(stationary_update(0, .104, false, error_state, &hard_p, &hard_v, false, false), 0);
		ASSERT_EQ(stationary_update(.1, .104, false, error_state, &soft_p, &soft_v, false, false), 0);
		ASSERT_EQ((fabs(hard_p) > .002), true);
		ASSERT_EQ((fabs(soft_p) < .0001), true);
		ASSERT_EQ((soft_v > hard_v && soft_v < .05), true);
	}
	return 0;
}

TEST(KalmanStationary, UncertainOpticalPoseDoesNotRestoreAHardStationaryOnset) {
	for (int error_state = 0; error_state <= 1; error_state++) {
		FLT hard_p, hard_v, soft_p, soft_v;
		ASSERT_EQ(stationary_update(0, .104, false, error_state, &hard_p, &hard_v, true, false), 0);
		ASSERT_EQ(stationary_update(.5, .104, false, error_state, &soft_p, &soft_v, true, false), 0);
		ASSERT_EQ((fabs(hard_p) > .002), true);
		ASSERT_EQ((fabs(soft_p) < .0001), true);
		ASSERT_EQ((soft_v > hard_v && soft_v < .05), true);
	}
	return 0;
}

TEST(KalmanStationary, UncertainOpticsWithoutLightRetainsThePreviousConstraint) {
	FLT hard_p, hard_v, soft_p, soft_v;
	ASSERT_EQ(stationary_update(0, .104, true, true, &hard_p, &hard_v, true, false), 0);
	ASSERT_EQ(stationary_update(.5, .104, true, true, &soft_p, &soft_v, true, false), 0);
	ASSERT_DOUBLE_EQ(hard_p, soft_p);
	ASSERT_DOUBLE_EQ(hard_v, soft_v);
	return 0;
}

TEST(KalmanStationary, SustainedRestReachesFullConstraint) {
	FLT hard_p, hard_v, soft_p, soft_v;
	ASSERT_EQ(stationary_update(0, .4, false, true, &hard_p, &hard_v, false, false), 0);
	ASSERT_EQ(stationary_update(.1, .4, false, true, &soft_p, &soft_v, false, false), 0);
	ASSERT_DOUBLE_EQ(hard_p, soft_p);
	ASSERT_DOUBLE_EQ(hard_v, soft_v);
	return 0;
}

TEST(KalmanStationary, MovingAndNoLightBehaviorIsPreserved) {
	for (int no_light = 0; no_light <= 1; no_light++) {
		FLT hard_p, hard_v, soft_p, soft_v;
		ASSERT_EQ(stationary_update(0, .05, no_light, true, &hard_p, &hard_v, false, false), 0);
		ASSERT_EQ(stationary_update(.1, .05, no_light, true, &soft_p, &soft_v, false, false), 0);
		ASSERT_DOUBLE_EQ(hard_p, soft_p);
		ASSERT_DOUBLE_EQ(hard_v, soft_v);
	}
	return 0;
}

struct micro_motion_result {
	FLT position_rms, angle_rms, position_gain, angle_gain;
	FLT velocity_rms, acceleration_rms, prediction_rms, position_phase_s, angle_phase_s;
	FLT optical_motion_until[2];
};

static int micro_motion_model(FLT frequency_hz, FLT ramp, int motion, FLT velocity_sigma, int quiet_pattern,
								 FLT optical_window, FLT optical_noise, struct micro_motion_result *result, int object_type, bool joint_bias) {
	SurviveContext ctx = {0};
	config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
	ctx.temporary_config_values = &temporary;
	survive_configi(&ctx, "kalman-joint-imu-bias", SC_SET | SC_OVERRIDE, joint_bias);
	survive_configi(&ctx, "kalman-minimize-state-space", SC_SET | SC_OVERRIDE, 0);
	survive_configf(&ctx, "kalman-zvu-stationary-ramp", SC_SET | SC_OVERRIDE, ramp);
	survive_configf(&ctx, "kalman-zvu-stationary-gyro-scale", SC_SET | SC_OVERRIDE, ramp > 0 ? .02 : 0);
	survive_configf(&ctx, "kalman-zvu-stationary-velocity-sigma", SC_SET | SC_OVERRIDE, velocity_sigma);
	if (optical_window >= 0)
		survive_configf(&ctx, "kalman-zvu-optical-motion-window", SC_SET | SC_OVERRIDE, optical_window);
	SurviveObject so = {.ctx = &ctx, .object_type = object_type, .timebase_hz = 48000000, .imu_freq = 250};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	so.tracker = &tracker;
	FLT p_error = 0, r_error = 0, p_sin = 0, p_cos = 0, r_sin = 0, r_cos = 0;
	FLT v_error = 0, acc_error = 0, prediction_error = 0;
	FLT omega = 2 * LINMATHPI * frequency_hz;
	int samples = 0;
	for (int i = 1; i <= 5000; i++) {
		FLT t = i * .004, phase = omega * t;
		bool translate = motion < 2, rotate = motion == 1 || motion == 2;
		FLT x = translate ? .002 * sin(phase) : 0, angle = rotate ? .02 * sin(phase) : 0;
		FLT acceleration = translate ? -.002 * omega * omega * sin(phase) : 0;
		survive_long_timecode tick = 48000000 + i * UINT64_C(192000);
		so.activations.last_light = so.activations.last_imu = tick;
		// Short apparent pauses are common while holding a tracker. The
		// ground truth here keeps moving smoothly through those detections.
		so.activations.last_movement = quiet_pattern == 1 ? 48000000 :
			quiet_pattern == 2 ? tick : tick - (i % 75) * UINT64_C(192000);
		PoserDataIMU imu = {.hdr = {.pt = POSERDATA_IMU, .timecode = tick},
			.accel = {cos(angle) * acceleration / 9.80665, -sin(angle) * acceleration / 9.80665, 1},
			.gyro = {0, 0, rotate ? .02 * omega * cos(phase) : 0}};
		if (i % 5 == 0) {
			SurvivePose truth = {.Pos = {x, 0, 0}, .Rot = {cos(angle / 2), 0, 0, sin(angle / 2)}};
			truth.Pos[0] += optical_noise * sin(17 * phase);
			truth.Pos[1] += optical_noise * cos(23 * phase);
			LinmathQuat noise;
			LinmathVec3d noise_rotation = {0, optical_noise * sin(19 * phase), 0};
			quatfromaxisanglemag(noise, noise_rotation);
			quatrotateabout(truth.Rot, noise, truth.Rot);
			PoserData pd = {.pt = POSERDATA_LIGHT_GEN2, .timecode = tick};
			survive_kalman_tracker_integrate_observation(&pd, &tracker, &truth, 0);
		}
		survive_kalman_tracker_integrate_imu(&tracker, &imu);
		if (t > 4) {
			FLT px = tracker.state.Pose.Pos[0];
			FLT a = 2 * atan2(tracker.state.Pose.Rot[3], tracker.state.Pose.Rot[0]);
			p_error += (px - x) * (px - x);
			r_error += (a - angle) * (a - angle);
			FLT dv = tracker.state.Velocity.Pos[0] - (translate ? .002 * omega * cos(phase) : 0);
			FLT da = tracker.state.Acc[0] - acceleration;
			FLT prediction = px + .02 * tracker.state.Velocity.Pos[0] + .0002 * tracker.state.Acc[0];
			FLT dp = prediction - (translate ? .002 * sin(phase + .02 * omega) : 0);
			v_error += dv * dv; acc_error += da * da; prediction_error += dp * dp;
			p_sin += px * sin(phase); p_cos += px * cos(phase);
			r_sin += a * sin(phase); r_cos += a * cos(phase);
			samples++;
		}
	}
	*result = (struct micro_motion_result){sqrt(p_error / samples), sqrt(r_error / samples),
		2 * hypot(p_sin, p_cos) / samples / .002, 2 * hypot(r_sin, r_cos) / samples / .02,
		sqrt(v_error / samples), sqrt(acc_error / samples), sqrt(prediction_error / samples),
		atan2(p_cos, p_sin) / omega, atan2(r_cos, r_sin) / omega,
		{tracker.optical_motion_until[0], tracker.optical_motion_until[1]}};
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

static int micro_motion_for_device(FLT frequency_hz, FLT ramp, int motion, FLT velocity_sigma, int quiet_pattern,
									 FLT optical_window, FLT optical_noise, struct micro_motion_result *result, int object_type) {
	return micro_motion_model(frequency_hz, ramp, motion, velocity_sigma, quiet_pattern,
		optical_window, optical_noise, result, object_type, false);
}

TEST(KalmanStationary, JointBiasPreservesSlowMotionAndTwoHertzDerivatives) {
	for (int frequency = 0; frequency < 2; frequency++)
		for (int motion = 0; motion < 2; motion++) {
			struct micro_motion_result result;
			ASSERT_EQ(micro_motion_model(frequency ? 2 : .5, .5, motion, .1, frequency ? 2 : 1,
				-1, 0, &result, SURVIVE_OBJECT_TYPE_CONTROLLER, true), 0);
			fprintf(stderr, "Joint bias %.1f Hz motion %d: gain %.6f/%.6f, pose %.6f mm, velocity %.6f mm/s, acceleration %.6f m/s2, prediction %.6f mm, phase %.6f/%.6f s\n",
				frequency ? 2. : .5, motion, result.position_gain, result.angle_gain, result.position_rms * 1000,
				result.velocity_rms * 1000, result.acceleration_rms, result.prediction_rms * 1000,
				result.position_phase_s, result.angle_phase_s);
			ASSERT_EQ((fabs(result.position_gain - 1) < .05), true);
			ASSERT_EQ((result.position_rms < .0001 && result.angle_rms < .001), true);
			ASSERT_EQ((result.velocity_rms < .0015 && result.acceleration_rms < (frequency ? .1 : .01)), true);
			ASSERT_EQ((result.prediction_rms < .00015 && fabs(result.position_phase_s) < .015), true);
			if (motion) {
				ASSERT_EQ((fabs(result.angle_gain - 1) < .05 && fabs(result.angle_phase_s) < .004), true);
			}
		}
	return 0;
}

static int micro_motion_at_frequency(FLT frequency_hz, FLT ramp, int motion, FLT velocity_sigma, int quiet_pattern,
									FLT optical_window, FLT optical_noise, struct micro_motion_result *result) {
	return micro_motion_for_device(frequency_hz, ramp, motion, velocity_sigma, quiet_pattern,
		optical_window, optical_noise, result, SURVIVE_OBJECT_TYPE_OTHER);
}

static int micro_motion_profile(FLT ramp, int motion, FLT velocity_sigma, bool persistent_quiet,
								FLT optical_window, FLT optical_noise, struct micro_motion_result *result) {
	return micro_motion_at_frequency(.5, ramp, motion, velocity_sigma, persistent_quiet ? 1 : 0,
		optical_window, optical_noise, result);
}

static int micro_motion(FLT ramp, bool rotate, FLT velocity_sigma, struct micro_motion_result *result) {
	return micro_motion_profile(ramp, rotate ? 1 : 0, velocity_sigma, false, 0, 0, result);
}

TEST(KalmanStationary, OpticalEvidencePreservesSustainedQuietMotion) {
	for (int motion = 0; motion < 3; motion++) {
		struct micro_motion_result result;
		// The 2 mm/0.5 Hz translation and .02 rad rotation are below the
		// detector's acceleration/gyro movement thresholds throughout. Keeping
		// its quiet classification for seconds must not erase that real motion.
		ASSERT_EQ(micro_motion_profile(.5, motion, .1, true, -1, 0, &result), 0);
		fprintf(stderr, "Sustained quiet motion %d: gains %.6f/%.6f, pose %.6f mm/%.6f deg, prediction %.6f mm, phase %.6f/%.6f s\n",
			motion, result.position_gain, result.angle_gain, result.position_rms * 1000,
			result.angle_rms * 180 / LINMATHPI, result.prediction_rms * 1000, result.position_phase_s, result.angle_phase_s);
		ASSERT_EQ((result.position_rms < .0001), true);
		ASSERT_EQ((result.angle_rms < .001), true);
		ASSERT_EQ((result.velocity_rms < .0015), true);
		ASSERT_EQ((result.acceleration_rms < .01), true);
		ASSERT_EQ((result.prediction_rms < .00015), true);
		if (motion < 2) {
			ASSERT_EQ((fabs(result.position_gain - 1) < .05), true);
			ASSERT_EQ((fabs(result.position_phase_s) < .015), true);
		}
		if (motion) {
			ASSERT_EQ((fabs(result.angle_gain - 1) < .05), true);
			ASSERT_EQ((fabs(result.angle_phase_s) < .004), true);
		}
	}
	return 0;
}

TEST(KalmanStationary, ControllersAndHMDsPreserveRealQuietMotion) {
	int types[] = {SURVIVE_OBJECT_TYPE_CONTROLLER, SURVIVE_OBJECT_TYPE_HMD};
	for (int type = 0; type < 2; type++)
		for (int persistent_quiet = 0; persistent_quiet < 2; persistent_quiet++) {
			struct micro_motion_result result;
			ASSERT_EQ(micro_motion_for_device(.5, .5, 1, .1, persistent_quiet, -1, 0, &result, types[type]), 0);
			ASSERT_EQ((fabs(result.position_gain - 1) < .05), true);
			ASSERT_EQ((fabs(result.angle_gain - 1) < .05), true);
			ASSERT_EQ((result.position_rms < .0001 && result.angle_rms < .001), true);
			ASSERT_EQ((result.velocity_rms < .0015 && result.acceleration_rms < .01), true);
			ASSERT_EQ((result.prediction_rms < .00015), true);
			ASSERT_EQ((fabs(result.position_phase_s) < .015 && fabs(result.angle_phase_s) < .004), true);
		}
	return 0;
}

TEST(KalmanStationary, StationaryOpticalNoiseDoesNotEstablishMotion) {
	struct micro_motion_result result;
	ASSERT_EQ(micro_motion_profile(.5, 3, .1, true, -1, .0007, &result), 0);
	ASSERT_DOUBLE_EQ(result.optical_motion_until[0], 0.);
	ASSERT_DOUBLE_EQ(result.optical_motion_until[1], 0.);
	ASSERT_EQ((result.position_rms < .0001), true);
	ASSERT_EQ((result.angle_rms < .001), true);
	ASSERT_EQ((result.prediction_rms < .00015), true);
	return 0;
}

TEST(KalmanStationary, UnavailableOpticalMotionDoesNotDisableBiasCalibrationForever) {
	SurviveContext ctx = {0};
	config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
	ctx.temporary_config_values = &temporary;
	SurviveObject so = {.ctx = &ctx, .timebase_hz = 48000000, .imu_freq = 250};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	tracker.model.t = tracker.imu_bias_model.t = 1;
	tracker.stats.obs_count = 16;
	tracker.optical_motion_start_time = 1;
	// Light is arriving, but no reliable optical velocity can be established.
	// A stationary IMU with a gyro bias must eventually regain its zero-rate
	// constraint rather than drifting for the entire unavailable interval.
	for (int i = 1; i <= 2500; i++) {
		survive_long_timecode tick = 48000000 + i * UINT64_C(192000);
		FLT time = tick / (FLT)so.timebase_hz;
		tracker.last_light_time = tracker.last_unreliable_pose_time = time;
		so.activations.last_light = so.activations.last_imu = tick;
		so.activations.last_movement = 48000000;
		PoserDataIMU imu = {.hdr = {.pt = POSERDATA_IMU, .timecode = tick},
			.accel = {0, 0, 1}, .gyro = {0, 0, .01}};
		survive_kalman_tracker_integrate_imu(&tracker, &imu);
	}
	ASSERT_EQ((fabs(tracker.state.Velocity.AxisAngleRot[2]) < .002), true);
	ASSERT_EQ((fabs(tracker.state.IMUBias.GyroBias[2] - .01) < .002), true);
	ASSERT_EQ(cn_is_finite(&tracker.model.P), true);
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

TEST(KalmanStationary, OpticalEvidencePreservesIntermittentQuietMotion) {
	for (int motion = 0; motion < 2; motion++) {
		struct micro_motion_result result;
		ASSERT_EQ(micro_motion_profile(.5, motion, .1, false, -1, 0, &result), 0);
		ASSERT_EQ((fabs(result.position_gain - 1) < .05), true);
		ASSERT_EQ((result.position_rms < .0001), true);
		ASSERT_EQ((result.angle_rms < .001), true);
		ASSERT_EQ((result.velocity_rms < .0015), true);
		ASSERT_EQ((result.acceleration_rms < .01), true);
		ASSERT_EQ((result.prediction_rms < .00015), true);
		ASSERT_EQ((fabs(result.position_phase_s) < .015), true);
		if (motion) {
			ASSERT_EQ((fabs(result.angle_gain - 1) < .05), true);
			ASSERT_EQ((fabs(result.angle_phase_s) < .004), true);
		}
	}
	return 0;
}

TEST(KalmanStationary, LowSpeedTwoHertzMotionRetainsDerivativeAndPredictionFidelity) {
	for (int motion = 0; motion < 2; motion++) {
		struct micro_motion_result result;
		// Low speed need not mean low frequency: a 2 mm, 2 Hz translation
		// peaks at only .025 m/s. Smoothing derivatives must retain that motion.
		ASSERT_EQ(micro_motion_at_frequency(2, .5, motion, .1, 2, -1, 0, &result), 0);
		fprintf(stderr, "2 Hz low-speed motion %d: gain %.6f, position %.6f mm, velocity %.6f mm/s, acceleration %.6f m/s2, prediction %.6f mm, phase %.6f s\n",
			motion, result.position_gain, result.position_rms * 1000, result.velocity_rms * 1000,
			result.acceleration_rms, result.prediction_rms * 1000, result.position_phase_s);
		ASSERT_EQ((fabs(result.position_gain - 1) < .05), true);
		ASSERT_EQ((result.position_rms < .0001), true);
		ASSERT_EQ((result.angle_rms < .001), true);
		ASSERT_EQ((result.velocity_rms < .0015), true);
		ASSERT_EQ((result.acceleration_rms < .1), true);
		ASSERT_EQ((result.prediction_rms < .00015), true);
		ASSERT_EQ((fabs(result.position_phase_s) < .015), true);
		if (motion) {
			ASSERT_EQ((fabs(result.angle_gain - 1) < .05), true);
			ASSERT_EQ((fabs(result.angle_phase_s) < .004), true);
		}
	}
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
	fprintf(stderr, "Micro-motion derivatives: velocity %.6f mm/s, acceleration %.6f m/s2, "
		"20ms prediction %.6f mm, phase %.6f %.6f s\n", soft.velocity_rms * 1000,
		soft.acceleration_rms, soft.prediction_rms * 1000, soft.position_phase_s, soft.angle_phase_s);
	// A 2 mm, 0.5 Hz motion must remain observable through short apparent pauses.
	// Bound derivative/prediction error as well as the pose's amplitude and phase.
	ASSERT_EQ((soft.velocity_rms < .0015), true);
	ASSERT_EQ((soft.acceleration_rms < .01), true);
	ASSERT_EQ((soft.prediction_rms < .00015), true);
	ASSERT_EQ((fabs(soft.position_phase_s) < .015), true);
	ASSERT_EQ((fabs(soft.angle_phase_s) < .004), true);
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

TEST(KalmanStationary, OpticalRecoveryReachesFullConstraintAfterSustainedRest) {
	FLT hard_p, hard_v, soft_p, soft_v;
	ASSERT_EQ(stationary_update(0, .8, false, true, &hard_p, &hard_v, true, false), 0);
	ASSERT_EQ(stationary_update(.5, .8, false, true, &soft_p, &soft_v, true, false), 0);
	ASSERT_DOUBLE_EQ(hard_p, soft_p);
	ASSERT_DOUBLE_EQ(hard_v, soft_v);
	return 0;
}

TEST(KalmanStationary, ExplicitControllerOptOutRetainsOriginalConstraint) {
	FLT hard_p, hard_v, soft_p, soft_v;
	ASSERT_EQ(stationary_update_with_options(0, .104, false, true, &hard_p, &hard_v, false, true, 0), 0);
	ASSERT_EQ(stationary_update_with_options(.5, .104, false, true, &soft_p, &soft_v, false, true, 0), 0);
	ASSERT_DOUBLE_EQ(hard_p, soft_p);
	ASSERT_DOUBLE_EQ(hard_v, soft_v);
	return 0;
}

TEST(KalmanStationary, ControllersHaveGradualStationaryOnsetByDefault) {
	FLT hard_p, hard_v, soft_p, soft_v;
	ASSERT_EQ(stationary_update(0, .104, false, true, &hard_p, &hard_v, true, true), 0);
	ASSERT_EQ(stationary_update(.5, .104, false, true, &soft_p, &soft_v, true, true), 0);
	ASSERT_EQ((fabs(hard_p) > .002), true);
	ASSERT_EQ((fabs(soft_p) < .0001), true);
	ASSERT_EQ((soft_v > hard_v && soft_v < .05), true);
	return 0;
}

TEST(KalmanStationary, DefaultTranslationNoiseComposesAcrossSplitPredictionIntervals) {
	SurviveObject so = {.timebase_hz = 48000000};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	struct SurviveKalmanTracker_Params params = tracker.params;
	survive_kalman_tracker_free(&tracker);
	SurviveKalmanModel state = {.Pose = {.Rot = {1, 0, 0, 0}}};
	CnMat x = cnVec(sizeof(state) / sizeof(FLT), (FLT *)&state);
	for (int error_state = 0; error_state < 2; error_state++) {
		int n = error_state ? 15 : 16;
		int index[] = {0,
			(error_state ? offsetof(SurviveKalmanErrorModel, Velocity.Pos) : offsetof(SurviveKalmanModel, Velocity.Pos)) / sizeof(FLT),
			(error_state ? offsetof(SurviveKalmanErrorModel, Acc) : offsetof(SurviveKalmanModel, Acc)) / sizeof(FLT)};
		CN_CREATE_STACK_MAT(whole, n, n);
		CN_CREATE_STACK_MAT(first, n, n);
		CN_CREATE_STACK_MAT(second, n, n);
		for (int split = 1; split < 4; split++) {
			FLT a = split * .001, b = .004 - a;
			survive_kalman_tracker_process_noise(&params, error_state, a + b, &x, &whole);
			survive_kalman_tracker_process_noise(&params, error_state, a, &x, &first);
			survive_kalman_tracker_process_noise(&params, error_state, b, &x, &second);
			FLT transition[3][3] = {{1, b, b * b / 2}, {0, 1, b}, {0, 0, 1}};
			// An optical update may split one IMU period into several predictions.
			// Q(a+b) = F(b) Q(a) F(b)^T + Q(b), independent of that cadence.
			for (int axis = 0; axis < 3; axis++) {
				for (int row = 0; row < 3; row++) {
					for (int col = 0; col < 3; col++) {
						FLT expected = cnMatrixGet(&whole, index[row] + axis, index[col] + axis);
						FLT actual = cnMatrixGet(&second, index[row] + axis, index[col] + axis);
						for (int i = 0; i < 3; i++)
							for (int j = 0; j < 3; j++)
								actual += transition[row][i] * transition[col][j] *
									cnMatrixGet(&first, index[i] + axis, index[j] + axis);
						ASSERT_EQ((fabs(actual - expected) < 1e-20 + fabs(expected) * 1e-6), true);
					}
				}
			}
		}
	}
	return 0;
}
