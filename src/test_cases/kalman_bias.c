#include "../survive_config.h"
#include "../survive_kalman_tracker.h"
#include "test_case.h"
#include <stddef.h>

TEST(KalmanBias, AccelerationCorrectionInvertsMeasurementModel) {
	SurviveKalmanTracker tracker = {0};
	const FLT acceleration[] = {.31, -.45, .85};
	const FLT scale[] = {.8, 1.2, 1.05};
	const FLT bias[] = {.03, -.02, .05};
	FLT measured[3], corrected[3];
	for (int i = 0; i < 3; i++) {
		tracker.state.IMUBias.AccScale[i] = scale[i];
		tracker.state.IMUBias.AccBias[i] = bias[i];
		// IMUMeasurementModel predicts measured = scale * acceleration + bias.
		measured[i] = scale[i] * acceleration[i] + bias[i];
	}
	survive_kalman_tracker_correct_imu(&tracker, corrected, measured);
	ASSERT_DOUBLE_ARRAY_EQ(3, corrected, acceleration);
	// Callers may correct their observation in place.
	survive_kalman_tracker_correct_imu(&tracker, measured, measured);
	ASSERT_DOUBLE_ARRAY_EQ(3, measured, acceleration);
	return 0;
}

static int bias_random_walk(int noise_model, bool joint) {
	SurviveContext ctx = {0};
	config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
	ctx.temporary_config_values = &temporary;
	survive_configi(&ctx, "kalman-joint-imu-bias", SC_SET | SC_OVERRIDE, joint);
	survive_configi(&ctx, "kalman-minimize-state-space", SC_SET | SC_OVERRIDE, 0);
	survive_configi(&ctx, "kalman-noise-model", SC_SET | SC_OVERRIDE, noise_model);
	SurviveObject so = {.ctx = &ctx};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	cnkalman_state_t *model = joint ? &tracker.model : &tracker.imu_bias_model;
	cn_set_zero(&model->P);
	model->t = 1;
	cnkalman_predict_state(1.02, model);
	CnMat P = survive_kalman_tracker_bias_covariance(&tracker);
	const int accel = offsetof(SurviveIMUBiasErrorModel, AccBias) / sizeof(FLT);
	const int gyro = offsetof(SurviveIMUBiasErrorModel, GyroBias) / sizeof(FLT);
	for (int axis = 0; axis < 3; axis++) {
		ASSERT_DOUBLE_EQ(cnMatrixGet(&P, accel + axis, accel + axis) /
		                 tracker.params.process_weight_acc_bias, .02);
		ASSERT_DOUBLE_EQ(cnMatrixGet(&P, gyro + axis, gyro + axis) /
		                 tracker.params.process_weight_gyro_bias, .02);
		ASSERT_DOUBLE_EQ(tracker.state.IMUBias.GyroBias[axis], 0.);
	}
	// Runtime configuration changes must reach the separate bias filter too.
	survive_configf(&ctx, "process-weight-gyro-bias", SC_SET | SC_OVERRIDE, 3e-8);
	cnkalman_predict_state(1.03, model);
	ASSERT_DOUBLE_EQ(cnMatrixGet(&P, gyro, gyro) / 1e-8, .05);
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

TEST(KalmanBias, BiasRandomWalkWithKinematicProcessNoise) { return bias_random_walk(0, false); }
TEST(KalmanBias, BiasRandomWalkWithDiagonalProcessNoise) { return bias_random_walk(1, false); }
TEST(KalmanBias, JointBiasRandomWalkWithKinematicProcessNoise) { return bias_random_walk(0, true); }
TEST(KalmanBias, JointBiasRandomWalkWithDiagonalProcessNoise) { return bias_random_walk(1, true); }

TEST(KalmanBias, JointFilterRetainsUnobservableBiasAndTransfersOpticalInformation) {
	SurviveContext ctx = {0};
	config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
	ctx.temporary_config_values = &temporary;
	survive_configi(&ctx, "kalman-joint-imu-bias", SC_SET | SC_OVERRIDE, 1);
	SurviveObject so = {.ctx = &ctx, .timebase_hz = 48000000, .imu_freq = 250};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	so.tracker = &tracker;
	ASSERT_EQ(tracker.model.state_cnt, 29);
	ASSERT_EQ(tracker.model.error_state_size, 27);
	ASSERT_EQ(tracker.imu_model.ks_cnt, 1);
	ASSERT_EQ(tracker.zvu_model.ks_cnt, 1);
	ASSERT_EQ(tracker.imu_bias_model.state_cnt, 0);
	CnMat bias = survive_kalman_tracker_bias_covariance(&tracker);
	ASSERT_EQ(bias.rows, 12);
	const int omega = offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot) / sizeof(FLT);
	const int gyro = offsetof(SurviveKalmanErrorModel, IMUBias.GyroBias) / sizeof(FLT);
	const int acc = offsetof(SurviveKalmanErrorModel, Acc) / sizeof(FLT);
	const int acc_bias = offsetof(SurviveKalmanErrorModel, IMUBias.AccBias) / sizeof(FLT);
	cn_set_zero(&tracker.model.P);
	const FLT variance = 1e-4, gravity = 9.80665;
	cnMatrixSet(&tracker.model.P, omega, omega, variance);
	cnMatrixSet(&tracker.model.P, gyro, gyro, variance);
	cnMatrixSet(&tracker.model.P, acc, acc, variance * gravity * gravity);
	cnMatrixSet(&tracker.model.P, acc_bias, acc_bias, variance);
	tracker.params = (struct SurviveKalmanTracker_Params){0};
	tracker.model.t = tracker.last_light_time = 1;
	tracker.stats.obs_count = 16;
	so.activations.last_imu = so.activations.last_light = so.activations.last_movement = 48000000;
	PoserDataIMU imu = {.hdr = {.pt = POSERDATA_IMU, .timecode = 48000000}, .accel = {0, 0, 1}};
	const int count = 100;
	for (int n = 0; n < count; n++) survive_kalman_tracker_integrate_imu(&tracker, &imu);
	// Repeated IMU observations identify omega+b and a/g+b, but cannot
	// identify their differences. Losing the negative cross terms would
	// incorrectly claim progressively smaller uncertainty in both terms.
	FLT difference_variance = cnMatrixGet(&tracker.model.P, omega, omega) +
		cnMatrixGet(&tracker.model.P, gyro, gyro) - 2 * cnMatrixGet(&tracker.model.P, omega, gyro);
	ASSERT_DOUBLE_EQ(difference_variance / variance, 2.);
	FLT expected_cross = -variance * variance / (2 * variance + tracker.gyro_var / count);
	ASSERT_DOUBLE_EQ(cnMatrixGet(&tracker.model.P, omega, gyro) / variance, expected_cross / variance);
	difference_variance = cnMatrixGet(&tracker.model.P, acc, acc) / (gravity * gravity) +
		cnMatrixGet(&tracker.model.P, acc_bias, acc_bias) - 2 * cnMatrixGet(&tracker.model.P, acc, acc_bias) / gravity;
	ASSERT_DOUBLE_EQ(difference_variance / variance, 2.);
	expected_cross = -variance * variance / (2 * variance + tracker.acc_var / count);
	ASSERT_DOUBLE_EQ(cnMatrixGet(&tracker.model.P, acc, acc_bias) / (gravity * variance), expected_cross / variance);
	SurvivePose optical = {.Pos = {.001, 0, 0}, .Rot = {cos(.0005), sin(.0005), 0, 0}};
	PoserData pd = {.pt = POSERDATA_LIGHT_GEN2, .timecode = 96000000};
	survive_kalman_tracker_integrate_observation(&pd, &tracker, &optical, NULL);
	fprintf(stderr, "Joint optical correction: omega %.9g, gyro bias %.9g, acceleration %.9g, accel bias %.9g\n",
		tracker.state.Velocity.AxisAngleRot[0], tracker.state.IMUBias.GyroBias[0],
		tracker.state.Acc[0], tracker.state.IMUBias.AccBias[0]);
	ASSERT_EQ((tracker.state.Velocity.AxisAngleRot[0] > 1e-5), true);
	ASSERT_EQ((tracker.state.IMUBias.GyroBias[0] < -1e-5), true);
	ASSERT_EQ((tracker.state.Acc[0] > 1e-5), true);
	ASSERT_EQ((tracker.state.IMUBias.AccBias[0] < -1e-5), true);
	ASSERT_EQ(cn_is_finite(&tracker.model.P), true);
	survive_kalman_tracker_free(&tracker);
	survive_kalman_tracker_init(&tracker, &so);
	bias = survive_kalman_tracker_bias_covariance(&tracker);
	ASSERT_EQ(bias.rows, 12);
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

TEST(KalmanBias, SmallCorrelationsAreInvariantToCovarianceScale) {
	for (int variant = 0; variant < 4; variant++) {
		bool full = variant % 2;
		FLT scale = variant < 2 ? 1 : 1e-12;
		FLT state[2] = {0};
		cnkalman_state_t model;
		cnkalman_state_init(&model, 2, NULL, NULL, NULL, state);
		model.t = 1;
		cnMatrixSet(&model.P, 0, 0, scale);
		cnMatrixSet(&model.P, 0, 1, .2 * scale);
		cnMatrixSet(&model.P, 1, 0, .2 * scale);
		cnMatrixSet(&model.P, 1, 1, .25 * scale);
		FLT h[] = {1, 1, 1, 1}, z[] = {1, 1};
		FLT noise_full[] = {.25, 0, 0, .25}, noise_diagonal[] = {.25, .25};
		for (int i = 0; i < 4; i++) noise_full[i] *= scale;
		for (int i = 0; i < 2; i++) noise_diagonal[i] *= scale;
		CnMat H = cnMat(2, 2, h), Z = cnVec(2, z);
		CnMat R = cnMat(2, full ? 2 : 1, full ? noise_full : noise_diagonal);
		cnkalman_meas_model_t measurement;
		cnkalman_meas_model_init(&model, "correlation", &measurement, NULL);
		measurement.term_criteria.max_iterations = 0;
		cnkalman_meas_model_predict_update(1, &measurement, &H, &Z, &R);
		// Two observations of x+b with independent noise .25 have effective
		// noise .125. Scaling P and R together must preserve this inference,
		// including the cross covariance, irrespective of coordinate units.
		ASSERT_DOUBLE_EQ(state[0], 48. / 71.);
		ASSERT_DOUBLE_EQ(state[1], 18. / 71.);
		ASSERT_DOUBLE_EQ(cnMatrixGet(&model.P, 0, 0) / scale, 67. / 355.);
		ASSERT_DOUBLE_EQ(cnMatrixGet(&model.P, 0, 1) / scale, -37. / 355.);
		ASSERT_DOUBLE_EQ(cnMatrixGet(&model.P, 1, 0) / scale, -37. / 355.);
		ASSERT_DOUBLE_EQ(cnMatrixGet(&model.P, 1, 1) / scale, 193. / 1420.);
		cnkalman_state_free(&model);
	}
	return 0;
}
