#include "../survive_config.h"
#include "../survive_kalman_tracker.h"
#include "test_case.h"
#include <stddef.h>

static int bias_random_walk(int noise_model) {
	SurviveContext ctx = {0};
	config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
	ctx.temporary_config_values = &temporary;
	survive_configi(&ctx, "kalman-minimize-state-space", SC_SET | SC_OVERRIDE, 0);
	survive_configi(&ctx, "kalman-noise-model", SC_SET | SC_OVERRIDE, noise_model);
	SurviveObject so = {.ctx = &ctx};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	cn_set_zero(&tracker.imu_bias_model.P);
	tracker.imu_bias_model.t = 1;
	cnkalman_predict_state(3, &tracker.imu_bias_model);
	const int accel = offsetof(SurviveIMUBiasErrorModel, AccBias) / sizeof(FLT);
	const int gyro = offsetof(SurviveIMUBiasErrorModel, GyroBias) / sizeof(FLT);
	for (int axis = 0; axis < 3; axis++) {
		ASSERT_DOUBLE_EQ(cnMatrixGet(&tracker.imu_bias_model.P, accel + axis, accel + axis) /
		                 tracker.params.process_weight_acc_bias, 2.);
		ASSERT_DOUBLE_EQ(cnMatrixGet(&tracker.imu_bias_model.P, gyro + axis, gyro + axis) /
		                 tracker.params.process_weight_gyro_bias, 2.);
		ASSERT_DOUBLE_EQ(tracker.state.IMUBias.GyroBias[axis], 0.);
	}
	// Runtime configuration changes must reach the separate bias filter too.
	survive_configf(&ctx, "process-weight-gyro-bias", SC_SET | SC_OVERRIDE, 3e-8);
	cnkalman_predict_state(4, &tracker.imu_bias_model);
	ASSERT_DOUBLE_EQ(cnMatrixGet(&tracker.imu_bias_model.P, gyro, gyro) / 1e-8, 5.);
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

TEST(KalmanBias, BiasRandomWalkWithKinematicProcessNoise) { return bias_random_walk(0); }
TEST(KalmanBias, BiasRandomWalkWithDiagonalProcessNoise) { return bias_random_walk(1); }
