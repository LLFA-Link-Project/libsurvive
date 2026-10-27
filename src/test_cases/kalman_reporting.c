#include "../survive_kalman_tracker.h"
#include "../generated/kalman_kinematics.gen.h"
#include "test_case.h"

static size_t pose_count;
static survive_long_timecode callback_tick;
static void receive_pose(SurviveObject *so, survive_long_timecode tick, const SurvivePose *pose) {
	pose_count++;
	callback_tick = so->OutPose_timecode = tick;
	so->OutPoseIMU = *pose;
}

// A stationary, valid filter state exercises the real report path, including
// non-IMU callers, without hardware, optimization, or a wall clock.
static int report_sequence(survive_long_timecode start, uint64_t period, FLT minimum,
                           size_t expected, bool interleave_light) {
	SurviveContext ctx = {.imuposeproc = receive_pose};
	SurviveObject so = {.ctx = &ctx, .timebase_hz = 48000000, .imu_freq = 250, .conf = "{}"};
	SurviveKalmanTracker tracker = {.so = &so, .min_report_time = minimum,
	                              .state = {.Pose = {.Rot = {1}}, .IMUBias = {.IMUCorrection = {1}}}};
	FLT covariance[13 * 13] = {0};
	for (int i = 0; i < 13; i++)
		covariance[i * 13 + i] = 1e-6;
	tracker.model.state_cnt = tracker.model.error_state_size = 13;
	tracker.model.state = cnVec(13, (FLT *)&tracker.state);
	tracker.model.P = cnMat(13, 13, covariance);
	pose_count = 0;
	callback_tick = 0;
	for (int i = 1; i <= 1000; i++) {
		PoserData pd = {.pt = POSERDATA_IMU, .timecode = start + i * period};
		tracker.model.t = pd.timecode / (FLT)so.timebase_hz;
		survive_kalman_tracker_report_state(&pd, &tracker);
		if (interleave_light) {
			// Same-tick optical corrections, out-of-order events, and updates
			// well inside a report period must not multiply callbacks.
			pd.pt = POSERDATA_LIGHT_GEN2;
			survive_kalman_tracker_report_state(&pd, &tracker);
			pd.timecode--;
			survive_kalman_tracker_report_state(&pd, &tracker);
			pd.timecode += period / 4 + 1;
			tracker.model.t = pd.timecode / (FLT)so.timebase_hz;
			survive_kalman_tracker_report_state(&pd, &tracker);
		}
	}
	ASSERT_EQ(pose_count, expected);
	ASSERT_EQ(tracker.stats.reported_poses, expected);
	ASSERT_EQ(callback_tick <= start + 1000 * period, true);
	ASSERT_DOUBLE_EQ(tracker.min_report_time, minimum);
	for (int i = 0; i < 13; i++)
		ASSERT_EQ(isfinite(((FLT *)&tracker.reported_state_variance)[i]), true);
	ASSERT_DOUBLE_EQ(so.OutPoseIMU.Rot[0], 1.);
	return 0;
}

TEST(KalmanReporting, NominalCadenceDoesNotHalveFastIMU) {
	const uint64_t periods[] = {191488, 192000, 192256}; // 3.989333 / 4.000 / 4.005333 ms
	const uint64_t starts[] = {0, (UINT64_C(1) << 32) - 1000000, UINT64_C(48000000) * 86400,
	                           UINT64_C(1) << 58};
	for (size_t i = 0; i < sizeof(periods) / sizeof(*periods); i++) {
		for (size_t j = 0; j < sizeof(starts) / sizeof(*starts); j++) {
			int result = report_sequence(starts[j], periods[i], -1, 1000, false);
			if (result) return result;
		}
	}
	return 0;
}

TEST(KalmanReporting, ExplicitMinimumStillLimitsOutput) {
	int result = report_sequence(UINT64_C(48000000) * 86400, 191488, .004, 500, false);
	if (result) return result;
	result = report_sequence(UINT64_C(48000000) * 86400, 192000, .004, 1000, false);
	if (result) return result;
	return report_sequence(UINT64_C(48000000) * 86400, 192256, .008, 500, false);
}

TEST(KalmanReporting, OpticalUpdatesDoNotDuplicateReports) {
	return report_sequence(UINT64_C(48000000) * 86400, 191488, -1, 1000, true);
}

TEST(KalmanReporting, ZeroMinimumHandlesSubUlpTickDifferences) {
	return report_sequence(UINT64_C(1) << 58, 1, 0, 1000, false);
}

TEST(KalmanReporting, DefaultStillThrottlesFasterUpdates) {
	return report_sequence(UINT64_C(48000000) * 86400, 96000, -1, 500, true);
}

TEST(KalmanPrediction, ZeroProcessNoiseDoesNotEraseEstimatedMotion) {
	SurviveKalmanModel state = {.Pose = {.Pos = {1, 2, 3}, .Rot = {.7, .2, -.3, .4}},
		.Velocity = {.Pos = {.01, -.02, .03}, .AxisAngleRot = {.2, -.1, .3}}, .Acc = {.03, -.04, .05}};
	quatnormalize(state.Pose.Rot, state.Pose.Rot);
	CnMat x = cnVec(16, (FLT *)&state);
	for (int acceleration_noise = 0; acceleration_noise <= 1; acceleration_noise++) {
		struct SurviveKalmanTracker_Params params = {.process_weight_acc = acceleration_noise};
		cnkalman_state_t model = {.user = &params};
		for (int step = 0; step <= 1; step++) {
			FLT dt = step * .02;
			SurviveKalmanModel output = {0};
			CnMat y = cnVec(16, (FLT *)&output);
			CN_CREATE_STACK_MAT(F, 16, 16);
			survive_kalman_tracker_predict_jac(dt, &model, &x, &y, &F);
			for (int axis = 0; axis < 3; axis++) {
				ASSERT_DOUBLE_EQ(output.Pose.Pos[axis], state.Pose.Pos[axis] + dt * state.Velocity.Pos[axis] +
					.5 * dt * dt * state.Acc[axis]);
				ASSERT_DOUBLE_EQ(output.Velocity.Pos[axis], state.Velocity.Pos[axis] + dt * state.Acc[axis]);
				ASSERT_DOUBLE_EQ(output.Acc[axis], state.Acc[axis]);
				ASSERT_DOUBLE_EQ(output.Velocity.AxisAngleRot[axis], state.Velocity.AxisAngleRot[axis]);
				ASSERT_DOUBLE_EQ(cnMatrixGet(&F, axis, 7 + axis), dt);
				ASSERT_DOUBLE_EQ(cnMatrixGet(&F, axis, 13 + axis), .5 * dt * dt);
			}
			LinmathQuat expected;
			survive_apply_ang_velocity(expected, state.Velocity.AxisAngleRot, dt, state.Pose.Rot);
			ASSERT_QUAT_EQ(output.Pose.Rot, expected);
		}
	}
	return 0;
}

TEST(KalmanPrediction, ShortIntervalsDoNotDiscardElapsedMotionAndUncertainty) {
	SurviveObject so = {.timebase_hz = 48000000};
	SurviveKalmanTracker whole, split;
	survive_kalman_tracker_init(&whole, &so);
	survive_kalman_tracker_init(&split, &so);
	whole.model.t = split.model.t = 1;
	whole.state.Pose.Pos[0] = split.state.Pose.Pos[0] = .1;
	whole.state.Velocity.Pos[0] = split.state.Velocity.Pos[0] = .01;
	whole.state.Acc[0] = split.state.Acc[0] = .03;
	whole.state.Velocity.AxisAngleRot[1] = split.state.Velocity.AxisAngleRot[1] = .1;
	whole.state.Velocity.AxisAngleRot[2] = split.state.Velocity.AxisAngleRot[2] = -.2;
	cn_set_zero(&whole.model.P);
	cn_set_zero(&split.model.P);
	cnkalman_predict_state(1.004, &whole.model);
	// Optical observations may fall just before or after an IMU sample.
	// Advancing the filter clock must include even a 50 us interval; dropping
	// it makes motion and uncertainty depend on how those events divide time.
	for (int i = 1; i <= 80; i++)
		cnkalman_predict_state(1 + i * .00005, &split.model);
	FLT tolerance = sizeof(FLT) == sizeof(float) ? 2e-6 : 1e-12;
	ASSERT_EQ((fabs(whole.state.Pose.Pos[0] - (.1 + .01 * .004 + .015 * .004 * .004)) < tolerance), true);
	ASSERT_EQ((fabs(split.state.Pose.Pos[0] - whole.state.Pose.Pos[0]) < tolerance), true);
	ASSERT_EQ((fabs(split.state.Velocity.Pos[0] - whole.state.Velocity.Pos[0]) < tolerance), true);
	ASSERT_QUAT_EQ(split.state.Pose.Rot, whole.state.Pose.Rot);
	int indices[] = {0, offsetof(SurviveKalmanErrorModel, Velocity.Pos) / sizeof(FLT),
		offsetof(SurviveKalmanErrorModel, Acc) / sizeof(FLT)};
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) {
			FLT expected = cnMatrixGet(&whole.model.P, indices[i], indices[j]);
			FLT actual = cnMatrixGet(&split.model.P, indices[i], indices[j]);
			FLT relative_tolerance = sizeof(FLT) == sizeof(float) ? 1e-4 : 1e-8;
			ASSERT_EQ((fabs(actual - expected) < 1e-20 + fabs(expected) * relative_tolerance), true);
		}
	ASSERT_DOUBLE_EQ(split.model.t, whole.model.t);
	survive_kalman_tracker_free(&whole);
	survive_kalman_tracker_free(&split);
	return 0;
}

TEST(KalmanPrediction, SignedTimePreservesConstantAccelerationTrajectory) {
	SurviveKalmanModel original = {.Pose = {.Pos = {.3, -.2, 1.1}, .Rot = {1}},
		.Velocity = {.Pos = {.02, -.03, .01}, .AxisAngleRot = {.01, -.02, .03}},
		.Acc = {.4, -.2, .1}, .IMUBias = {.AccScale = {1, 1, 1}, .IMUCorrection = {1}}};
	FLT intervals[] = {-.04, -.003, 0, .003, .04};
	for (int n = 0; n < sizeof(intervals) / sizeof(intervals[0]); n++) {
		FLT dt = intervals[n];
		SurviveKalmanModel predicted, restored;
		SurviveKalmanModelPredict(&predicted, dt, &original);
		SurviveKalmanModelPredict(&restored, -dt, &predicted);
		CN_CREATE_STACK_MAT(F, sizeof(original) / sizeof(FLT), sizeof(original) / sizeof(FLT));
		CN_CREATE_STACK_VEC(T, sizeof(original) / sizeof(FLT));
		SurviveKalmanModelPredict_jac_kalman_model(&F, dt, &original);
		SurviveKalmanModelPredict_jac_t(&T, dt, &original);
		for (int axis = 0; axis < 3; axis++) {
			FLT position = original.Pose.Pos[axis] + dt * original.Velocity.Pos[axis] +
				.5 * dt * dt * original.Acc[axis];
			ASSERT_DOUBLE_EQ(predicted.Pose.Pos[axis], position);
			ASSERT_DOUBLE_EQ(restored.Pose.Pos[axis], original.Pose.Pos[axis]);
			ASSERT_DOUBLE_EQ(restored.Velocity.Pos[axis], original.Velocity.Pos[axis]);
			int accel = offsetof(SurviveKalmanModel, Acc) / sizeof(FLT) + axis;
			ASSERT_DOUBLE_EQ(cnMatrixGet(&F, axis, accel), .5 * dt * dt);
			ASSERT_DOUBLE_EQ(cnMatrixGet(&T, axis, 0), predicted.Velocity.Pos[axis]);
		}
		ASSERT_QUAT_EQ(restored.Pose.Rot, original.Pose.Rot);
	}
	return 0;
}

TEST(KalmanPrediction, ProcessNoiseIsIndependentOfWorldAxes) {
	struct SurviveKalmanTracker_Params params = {.process_weight_acc = 1, .process_weight_ang_velocity = 1};
	SurviveKalmanModel original = {.Pose = {.Rot = {1}}}, rotated = original;
	LinmathVec3d axes = {.6, -.8, 1.1};
	quatfromaxisanglemag(rotated.Pose.Rot, axes);
	CnMat x = cnVec(16, (FLT *)&original), xr = cnVec(16, (FLT *)&rotated);
	CN_CREATE_STACK_MAT(Q, 15, 15);
	CN_CREATE_STACK_MAT(Qr, 15, 15);
	survive_kalman_tracker_process_noise(&params, true, .02, &x, &Q);
	survive_kalman_tracker_process_noise(&params, true, .02, &xr, &Qr);
	CN_CREATE_STACK_MAT(frame, 15, 15);
	// Rotation error is local (q_nominal^-1*q_true); translation, velocity,
	// angular velocity and acceleration use world axes. Changing the world
	// frame must rotate only those world-coordinate error components.
	for (int i = 3; i < 6; i++) cnMatrixSet(&frame, i, i, 1);
	int starts[] = {0, 6, 9, 12};
	for (int axis = 0; axis < 3; axis++) {
		LinmathVec3d basis = {0}, column;
		basis[axis] = 1;
		quatrotatevector(column, rotated.Pose.Rot, basis);
		for (int group = 0; group < 4; group++)
			for (int row = 0; row < 3; row++)
				cnMatrixSet(&frame, starts[group] + row, starts[group] + axis, column[row]);
	}
	CN_CREATE_STACK_MAT(expected, 15, 15);
	cn_ABAt_add(&expected, &frame, &Q, NULL);
	for (int i = 0; i < 15; i++)
		for (int j = 0; j < 15; j++) {
			FLT difference = cnMatrixGet(&Qr, i, j) - cnMatrixGet(&expected, i, j);
			ASSERT_EQ((fabs(difference) < 1e-11), true);
		}
	return 0;
}

TEST(KalmanPrediction, LocalOrientationUncertaintyFollowsThePredictedNominalPose) {
	SurviveObject so = {.timebase_hz = 48000000};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	memset(&tracker.params, 0, sizeof(tracker.params));
	tracker.model.t = 1;
	LinmathVec3d initial_rotation = {.7, -.3, .4};
	quatfromaxisanglemag(tracker.state.Pose.Rot, initial_rotation);
	FLT rates[] = {.01, 1};
	FLT intervals[] = {-.02, .004, .02};
	for (int rate = 0; rate < 2; rate++) {
		tracker.state.Velocity.AxisAngleRot[0] = 2 * rates[rate];
		tracker.state.Velocity.AxisAngleRot[1] = -3 * rates[rate];
		tracker.state.Velocity.AxisAngleRot[2] = 4 * rates[rate];
		for (int interval = 0; interval < 3; interval++) {
			cn_set_zero(&tracker.model.P);
			for (int axis = 0; axis < 3; axis++)
				cnMatrixSet(&tracker.model.P, 3 + axis, 3 + axis, .01 * (axis + 1));
			cnMatrixSet(&tracker.model.P, 3, 4, .003);
			cnMatrixSet(&tracker.model.P, 4, 3, .003);
			SurviveKalmanModel predicted = {0};
			CnMat state = cnVec(tracker.model.state_cnt, (FLT *)&predicted);
			CN_CREATE_STACK_MAT(P, tracker.model.P.rows, tracker.model.P.cols);
			cnkalman_extrapolate_state(1 + intervals[interval], &tracker.model, &state, &P);
			// With world angular velocity known exactly, a local orientation
			// error is unchanged: Exp(w*dt)*q*Exp(error) = q_pred*Exp(error).
			// Its covariance must be expressed about q_pred, not the old q.
			for (int row = 0; row < P.rows; row++)
				for (int col = 0; col < P.cols; col++) {
					FLT difference = cnMatrixGet(&P, row, col) - cnMatrixGet(&tracker.model.P, row, col);
					FLT tolerance = sizeof(FLT) == sizeof(float) ? 1e-7 : 1e-12;
					ASSERT_EQ((fabs(difference) < tolerance), true);
				}
		}
	}
	survive_kalman_tracker_free(&tracker);
	return 0;
}

static SurviveKalmanModel physical_prediction(SurviveKalmanModel state, FLT dt) {
	for (int axis = 0; axis < 3; axis++) {
		state.Pose.Pos[axis] += dt * state.Velocity.Pos[axis] + .5 * dt * dt * state.Acc[axis];
		state.Velocity.Pos[axis] += dt * state.Acc[axis];
	}
	LinmathVec3d rotation;
	scale3d(rotation, state.Velocity.AxisAngleRot, dt);
	LinmathQuat increment;
	quatfromaxisanglemag(increment, rotation);
	quatrotateabout(state.Pose.Rot, increment, state.Pose.Rot);
	return state;
}

TEST(KalmanPrediction, PredictedCovarianceMatchesPhysicalStatePerturbations) {
	SurviveObject so = {.timebase_hz = 48000000};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	memset(&tracker.params, 0, sizeof(tracker.params));
	tracker.model.t = 1;
	ASSERT_EQ(tracker.model.state_cnt, 16);
	tracker.state.Pose.Pos[0] = .3;
	tracker.state.Velocity.Pos[1] = -.4;
	tracker.state.Acc[2] = .7;
	LinmathVec3d initial_rotation = {.7, -.3, .4};
	quatfromaxisanglemag(tracker.state.Pose.Rot, initial_rotation);
	copy3d(tracker.state.Velocity.AxisAngleRot, (LinmathVec3d){2, -3, 4});
	cn_set_zero(&tracker.model.P);
	for (int i = 0; i < 15; i++) cnMatrixSet(&tracker.model.P, i, i, .001 * (i + 1));
	cnMatrixSet(&tracker.model.P, 1, 4, .0007);
	cnMatrixSet(&tracker.model.P, 4, 1, .0007);
	cnMatrixSet(&tracker.model.P, 3, 10, -.0009);
	cnMatrixSet(&tracker.model.P, 10, 3, -.0009);
	FLT intervals[] = {-.03, .004, .03};
	for (int interval = 0; interval < 3; interval++) {
		FLT dt = intervals[interval], epsilon = sizeof(FLT) == sizeof(float) ? .002 : 1e-6;
		SurviveKalmanModel nominal = physical_prediction(tracker.state, dt), actual = {0};
		CnMat state = cnVec(tracker.model.state_cnt, (FLT *)&actual);
		CN_CREATE_STACK_MAT(P, 15, 15);
		cnkalman_extrapolate_state(1 + dt, &tracker.model, &state, &P);
		ASSERT_DOUBLE_ARRAY_EQ(16, ((FLT *)&actual), ((FLT *)&nominal));
		CN_CREATE_STACK_MAT(J, 15, 15);
		for (int col = 0; col < 15; col++) {
			FLT errors[2][15];
			for (int side = 0; side < 2; side++) {
				FLT step = side ? epsilon : -epsilon;
				SurviveKalmanModel perturbed = tracker.state;
				if (col >= 3 && col < 6) {
					LinmathVec3d rotation = {0};
					rotation[col - 3] = step;
					LinmathQuat increment;
					quatfromaxisanglemag(increment, rotation);
					quatrotateabout(perturbed.Pose.Rot, perturbed.Pose.Rot, increment);
				} else {
					((FLT *)&perturbed)[col < 3 ? col : col + 1] += step;
				}
				perturbed = physical_prediction(perturbed, dt);
				sub3d(errors[side], perturbed.Pose.Pos, nominal.Pose.Pos);
				LinmathQuat inverse, relative;
				quatgetconjugate(inverse, nominal.Pose.Rot);
				quatrotateabout(relative, inverse, perturbed.Pose.Rot);
				// The relative rotation is infinitesimal; 2*q.xyz has
				// the same first derivative as its rotation vector.
				scale3d(errors[side] + 3, relative + 1, 2);
				sub3d(errors[side] + 6, perturbed.Velocity.Pos, nominal.Velocity.Pos);
				sub3d(errors[side] + 9, perturbed.Velocity.AxisAngleRot, nominal.Velocity.AxisAngleRot);
				sub3d(errors[side] + 12, perturbed.Acc, nominal.Acc);
			}
			for (int row = 0; row < 15; row++)
				cnMatrixSet(&J, row, col, (errors[1][row] - errors[0][row]) / (2 * epsilon));
		}
		CN_CREATE_STACK_MAT(expected, 15, 15);
		cn_ABAt_add(&expected, &J, &tracker.model.P, NULL);
		for (int row = 0; row < 15; row++)
			for (int col = 0; col < 15; col++) {
				FLT difference = cnMatrixGet(&P, row, col) - cnMatrixGet(&expected, row, col);
				FLT tolerance = sizeof(FLT) == sizeof(float) ? 1e-5 : 1e-10;
				ASSERT_EQ((fabs(difference) < tolerance), true);
			}
	}
	survive_kalman_tracker_free(&tracker);
	return 0;
}
