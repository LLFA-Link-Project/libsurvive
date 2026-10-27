#include "../survive_config.h"
#include "../survive_kalman_tracker.h"
#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void multiply(FLT out[4], const FLT a[4], const FLT b[4]) {
	FLT q[4] = {
		a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
		a[0] * b[2] + a[2] * b[0] + a[3] * b[1] - a[1] * b[3], a[0] * b[3] + a[3] * b[0] + a[1] * b[2] - a[2] * b[1]};
	double n = 0;
	for (int i = 0; i < 4; i++)
		n += q[i] * q[i];
	n = sqrt(n);
	for (int i = 0; i < 4; i++)
		out[i] = q[i] / n;
}
static void exponential(FLT out[4], const FLT v[3]) {
	double norm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]), s = norm > 1e-12 ? sin(norm / 2) / norm : .5;
	out[0] = cos(norm / 2);
	for (int i = 0; i < 3; i++)
		out[i + 1] = v[i] * s;
}
static void rotate(FLT out[3], const FLT q[4], const FLT v[3]) {
	FLT u[3] = {q[2] * v[2] - q[3] * v[1], q[3] * v[0] - q[1] * v[2], q[1] * v[1] - q[2] * v[0]};
	FLT second[3] = {q[2] * u[2] - q[3] * u[1], q[3] * u[0] - q[1] * u[2], q[1] * u[1] - q[2] * u[0]};
	for (int i = 0; i < 3; i++)
		out[i] = v[i] + 2 * q[0] * u[i] + 2 * second[i];
}
// Independent physical map: carry the previous rate to the acquisition time,
// then express the new body-frame sample in world coordinates.
static void physical_map(SurviveKalmanModel *out, const SurviveKalmanModel *old, const FLT gyro[3], FLT dt) {
	*out = *old;
	FLT body[3], step[3], dq[4];
	for (int i = 0; i < 3; i++)
		body[i] = gyro[i] - old->IMUBias.GyroBias[i];
	for (int i = 0; i < 3; i++) {
		step[i] = old->Velocity.AxisAngleRot[i] * dt;
		out->Pose.Pos[i] = old->Pose.Pos[i] + dt * old->Velocity.Pos[i] + .5 * dt * dt * old->Acc[i];
		out->Velocity.Pos[i] = old->Velocity.Pos[i] + dt * old->Acc[i];
	}
	exponential(dq, step);
	multiply(out->Pose.Rot, dq, old->Pose.Rot);
	rotate(out->Velocity.AxisAngleRot, out->Pose.Rot, body);
}
TEST(KalmanGyroControl, CovarianceMatchesIndependentPhysicalPerturbations) {
	for (int sign = 0; sign < 2; sign++) {
		SurviveContext ctx = {0};
		config_group config = {.write_lock = OGCreateMutex(), .ctx = &ctx};
		ctx.temporary_config_values = &config;
		survive_configi(&ctx, "kalman-joint-imu-bias", SC_SET | SC_OVERRIDE, 1);
		survive_configi(&ctx, "kalman-gyro-control", SC_SET | SC_OVERRIDE, 1);
		SurviveObject so = {
			.ctx = &ctx, .timebase_hz = 48000000, .imu_freq = 250, .object_type = SURVIVE_OBJECT_TYPE_CONTROLLER};
		SurviveKalmanTracker tracker;
		survive_kalman_tracker_init(&tracker, &so);
		so.tracker = &tracker;
		ASSERT_EQ(tracker.model.state_cnt, 29);
		ASSERT_EQ(tracker.model.error_state_size, 27);
		tracker.acc_var = -1;
		tracker.params = (struct SurviveKalmanTracker_Params){0};
		tracker.model.t = tracker.last_light_time = 1;
		tracker.stats.obs_count = 16;
		so.activations.last_imu = so.activations.last_light = so.activations.last_movement = 48000000;
		PoserDataIMU imu = {.hdr = {.pt = POSERDATA_IMU, .timecode = 48000000}, .accel = {0, 0, 1}};
		survive_kalman_tracker_integrate_imu(&tracker, &imu);
		SurviveKalmanModel old = {.Pose = {.Pos = {.2, -.1, .4}},
								  .Velocity = {.Pos = {.1, .2, -.15}, .AxisAngleRot = {.3, -.2, .4}},
								  .Acc = {.07, -.06, .02},
								  .IMUBias = {.AccScale = {1, 1, 1},
											  .IMUCorrection = {1},
											  .GyroBias = {.002, -.003, .001},
											  .AccBias = {.001, -.002, .0005}}};
		FLT axis[3] = {.4, -.3, .2};
		exponential(old.Pose.Rot, axis);
		if (sign)
			for (int i = 0; i < 4; i++)
				old.Pose.Rot[i] = -old.Pose.Rot[i];
		tracker.state = old;
		double L[27 * 27] = {0}, P[27 * 27] = {0};
		for (int i = 0; i < 27; i++)
			for (int j = 0; j <= i; j++)
				if ((i < 18 || i > 20) && (j < 18 || j > 20))
					L[27 * i + j] = i == j ? .002 + .0001 * i : .0001 * sin((i + 1) * (j + 3));
		for (int i = 0; i < 27; i++)
			for (int j = 0; j < 27; j++) {
				for (int k = 0; k < 27; k++)
					P[27 * i + j] += L[27 * i + k] * L[27 * j + k];
				cnMatrixSet(&tracker.model.P, i, j, P[27 * i + j]);
			}
		const FLT dt = .004, epsilon = 1e-6, gyro[3] = {.31, -.17, .21};
		imu.hdr.timecode += 192000;
		for (int i = 0; i < 3; i++)
			imu.gyro[i] = gyro[i];
		so.activations.last_imu = so.activations.last_light = so.activations.last_movement = imu.hdr.timecode;
		survive_kalman_tracker_integrate_imu(&tracker, &imu);
		SurviveKalmanModel mean;
		physical_map(&mean, &old, gyro, dt);
		double mean_error = 0;
		for (int i = 0; i < 29; i++)
			mean_error = fmax(mean_error, fabs(((FLT *)&mean)[i] - ((FLT *)&tracker.state)[i]));
		FLT J[27 * 27] = {0}, N[27 * 3] = {0};
		CnMat prior = cnVec(29, (FLT *)&old);
		for (int col = 0; col < 30; col++) {
			FLT values[2][27];
			for (int side = 0; side < 2; side++) {
				FLT delta[27] = {0}, measurement[3] = {gyro[0], gyro[1], gyro[2]};
				SurviveKalmanModel perturbed = old, result;
				if (col < 27) {
					delta[col] = (side ? 1 : -1) * epsilon;
					CnMat e = cnVec(27, delta), x = cnVec(29, (FLT *)&perturbed);
					tracker.model.Update_fn(tracker.model.user, &prior, &e, &x, NULL);
				} else
					measurement[col - 27] += (side ? 1 : -1) * epsilon;
				physical_map(&result, &perturbed, measurement, dt);
				CnMat point = cnVec(29, (FLT *)&result), error = cnVec(27, values[side]);
				tracker.model.ErrorState_fn(tracker.model.user, &tracker.model.state, &point, &error, NULL);
			}
			for (int row = 0; row < 27; row++) {
				FLT value = (values[1][row] - values[0][row]) / (2 * epsilon);
				if (col < 27)
					J[27 * row + col] = value;
				else
					N[3 * row + col - 27] = value;
			}
		}
		double difference = 0, norm = 0;
		for (int i = 0; i < 27; i++)
			for (int j = 0; j < 27; j++) {
				double expected = 0;
				for (int k = 0; k < 27; k++)
					for (int l = 0; l < 27; l++)
						expected += J[27 * i + k] * P[27 * k + l] * J[27 * j + l];
				for (int k = 0; k < 3; k++)
					expected += tracker.gyro_var * N[3 * i + k] * N[3 * j + k];
				double actual = cnMatrixGet(&tracker.model.P, i, j);
				difference += (actual - expected) * (actual - expected);
				norm += expected * expected;
			}
		double relative = sqrt(difference / norm);
		printf("{\"sign\":%d,\"maximum_mean_error\":%.17g,\"relative_covariance_error\":%.17g}\n", sign, mean_error,
			   relative);
		survive_kalman_tracker_free(&tracker);
		for (int i = 0; i < config.used_entries; i++) {
			free(config.config_entries[i].tag);
			free(config.config_entries[i].data);
		}
		free(config.config_entries);
		OGDeleteMutex(config.write_lock);
		ASSERT_EQ(isfinite(relative), true);
		ASSERT_GT(1e-10, mean_error);
		ASSERT_GT(1e-6, relative);
	}
	return 0;
}

TEST(KalmanGyroControl, IntermediatePredictionPreservesStateAndCovariance) {
	FLT states[2][29], covariances[2][27 * 27];
	for (int split = 0; split < 2; split++) {
		SurviveContext ctx = {0};
		config_group config = {.write_lock = OGCreateMutex(), .ctx = &ctx};
		ctx.temporary_config_values = &config;
		survive_configi(&ctx, "kalman-joint-imu-bias", SC_SET | SC_OVERRIDE, 1);
		survive_configi(&ctx, "kalman-gyro-control", SC_SET | SC_OVERRIDE, 1);
		SurviveObject so = {
			.ctx = &ctx, .timebase_hz = 48000000, .imu_freq = 250, .object_type = SURVIVE_OBJECT_TYPE_CONTROLLER};
		SurviveKalmanTracker tracker;
		survive_kalman_tracker_init(&tracker, &so);
		so.tracker = &tracker;
		tracker.acc_var = -1;
		tracker.params = (struct SurviveKalmanTracker_Params){0};
		tracker.model.t = tracker.last_light_time = 1;
		tracker.stats.obs_count = 16;
		so.activations.last_imu = so.activations.last_light = so.activations.last_movement = 48000000;
		PoserDataIMU imu = {.hdr = {.pt = POSERDATA_IMU, .timecode = 48000000}, .accel = {0, 0, 1}};
		survive_kalman_tracker_integrate_imu(&tracker, &imu);
		FLT q[4] = {.92, .2, -.12, .29}, norm = 0;
		for (int i = 0; i < 4; i++)
			norm += q[i] * q[i];
		norm = sqrt(norm);
		for (int i = 0; i < 4; i++)
			tracker.state.Pose.Rot[i] = q[i] / norm;
		for (int i = 0; i < 3; i++) {
			tracker.state.Velocity.AxisAngleRot[i] = i == 1 ? -.3 : .4;
			tracker.state.Velocity.Pos[i] = .1;
			tracker.state.Acc[i] = .03;
		}
		for (int i = 0; i < 27; i++)
			for (int j = 0; j < 27; j++)
				cnMatrixSet(&tracker.model.P, i, j, i == j && (i < 18 || i > 20) ? 1e-4 : 0);
		if (split)
			cnkalman_predict_state(1.002, &tracker.model);
		imu.hdr.timecode += 192000;
		imu.gyro[0] = .31;
		imu.gyro[1] = -.17;
		imu.gyro[2] = .21;
		so.activations.last_imu = so.activations.last_light = so.activations.last_movement = imu.hdr.timecode;
		survive_kalman_tracker_integrate_imu(&tracker, &imu);
		for (int i = 0; i < 29; i++)
			states[split][i] = ((FLT *)&tracker.state)[i];
		for (int i = 0; i < 27; i++)
			for (int j = 0; j < 27; j++)
				covariances[split][27 * i + j] = cnMatrixGet(&tracker.model.P, i, j);
		survive_kalman_tracker_free(&tracker);
		for (int i = 0; i < config.used_entries; i++) {
			free(config.config_entries[i].tag);
			free(config.config_entries[i].data);
		}
		free(config.config_entries);
		OGDeleteMutex(config.write_lock);
	}
	double state_error = 0, covariance_error = 0;
	for (int i = 0; i < 29; i++)
		state_error = fmax(state_error, fabs(states[0][i] - states[1][i]));
	for (int i = 0; i < 27 * 27; i++)
		covariance_error = fmax(covariance_error, fabs(covariances[0][i] - covariances[1][i]));
	printf("{\"maximum_state_partition_difference\":%.17g,\"maximum_covariance_partition_difference\":%.17g}\n",
		   state_error, covariance_error);
	ASSERT_GT(1e-10, state_error);
	ASSERT_GT(1e-10, covariance_error);
	return 0;
}

TEST(KalmanGyroControl, UnsupportedConfigurationsKeepTheMeasurementFilter) {
	for (int variant = 0; variant < 9; variant++) {
		SurviveContext ctx = {0};
		config_group config = {.write_lock = OGCreateMutex(), .ctx = &ctx};
		ctx.temporary_config_values = &config;
		survive_configi(&ctx, "kalman-gyro-control", SC_SET | SC_OVERRIDE, variant != 1);
		if (variant == 2)
			survive_configi(&ctx, "use-imu", SC_SET | SC_OVERRIDE, 0);
		if (variant == 3)
			survive_configi(&ctx, "kalman-use-error-space", SC_SET | SC_OVERRIDE, 0);
		if (variant == 4)
			survive_configi(&ctx, "kalman-noise-model", SC_SET | SC_OVERRIDE, 1);
		if (variant == 5)
			survive_configf(&ctx, "kalman-initial-imu-variance", SC_SET | SC_OVERRIDE, 1e-6);
		if (variant == 6)
			survive_configi(&ctx, "kalman-obj-imu-adaptive", SC_SET | SC_OVERRIDE, 1);
		if (variant == 7)
			survive_configi(&ctx, "kalman-obj-imu-error-state-model", SC_SET | SC_OVERRIDE, 0);
		if (variant == 8)
			survive_configi(&ctx, "use-kalman", SC_SET | SC_OVERRIDE, 0);
		SurviveObject so = {.ctx = &ctx, .timebase_hz = 48000000};
		SurviveKalmanTracker tracker;
		survive_kalman_tracker_init(&tracker, &so);
		bool enabled = tracker.gyro_control_mode;
		int dimension = tracker.model.state_cnt;
		survive_kalman_tracker_free(&tracker);
		for (int i = 0; i < config.used_entries; i++) {
			ASSERT_EQ(config.config_entries[i].update_list, NULL);
			free(config.config_entries[i].tag);
			free(config.config_entries[i].data);
		}
		free(config.config_entries);
		OGDeleteMutex(config.write_lock);
		ASSERT_EQ(enabled, (variant == 0));
		ASSERT_EQ(dimension, (variant == 0 ? 29 : 16));
	}
	return 0;
}

static int roll_motion(FLT frequency, FLT *gain, FLT *phase, FLT *acceleration_rms, FLT *position_rms,
					   FLT *prediction_rms) {
	SurviveContext ctx = {0};
	config_group config = {.write_lock = OGCreateMutex(), .ctx = &ctx};
	ctx.temporary_config_values = &config;
	survive_configi(&ctx, "kalman-gyro-control", SC_SET | SC_OVERRIDE, 1);
	SurviveObject so = {
		.ctx = &ctx, .timebase_hz = 48000000, .imu_freq = 250, .object_type = SURVIVE_OBJECT_TYPE_CONTROLLER};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	so.tracker = &tracker;
	double sine = 0, cosine = 0, acceleration_error = 0, position_error = 0, prediction_error = 0;
	int count = 0;
	for (int sample = 1; sample <= 5000; sample++) {
		FLT t = sample * .004, omega = 2 * LINMATHPI * frequency;
		FLT angle = .02 * sin(omega * t);
		uint64_t tick = UINT64_C(48000000) + sample * UINT64_C(192000);
		so.activations.last_light = so.activations.last_imu = tick;
		// The detector remains quiet, but the true controller keeps rolling.
		so.activations.last_movement = 48000000;
		PoserDataIMU imu = {.hdr = {.pt = POSERDATA_IMU, .timecode = tick},
							.accel = {0, sin(angle), cos(angle)},
							.gyro = {.02 * omega * cos(omega * t), 0, 0}};
		if (sample % 5 == 0) {
			SurvivePose observed = {.Rot = {cos(angle / 2), sin(angle / 2), 0, 0}};
			PoserData pd = {.pt = POSERDATA_LIGHT_GEN2, .timecode = tick};
			survive_kalman_tracker_integrate_observation(&pd, &tracker, &observed, NULL);
		}
		survive_kalman_tracker_integrate_imu(&tracker, &imu);
		if (t > 4) {
			FLT actual = 2 * atan2(tracker.state.Pose.Rot[1], tracker.state.Pose.Rot[0]);
			sine += actual * sin(omega * t);
			cosine += actual * cos(omega * t);
			for (int axis = 0; axis < 3; axis++) {
				double prediction = tracker.state.Pose.Pos[axis] + .02 * tracker.state.Velocity.Pos[axis] +
									.0002 * tracker.state.Acc[axis];
				acceleration_error += tracker.state.Acc[axis] * tracker.state.Acc[axis];
				position_error += tracker.state.Pose.Pos[axis] * tracker.state.Pose.Pos[axis];
				prediction_error += prediction * prediction;
			}
			count++;
		}
	}
	*gain = 2 * hypot(sine, cosine) / count / .02;
	*phase = atan2(cosine, sine) / (2 * LINMATHPI * frequency);
	*acceleration_rms = sqrt(acceleration_error / count);
	*position_rms = sqrt(position_error / count);
	*prediction_rms = sqrt(prediction_error / count);
	survive_kalman_tracker_free(&tracker);
	for (int i = 0; i < config.used_entries; i++) {
		free(config.config_entries[i].tag);
		free(config.config_entries[i].data);
	}
	free(config.config_entries);
	OGDeleteMutex(config.write_lock);
	return 0;
}

TEST(KalmanGyroControl, SlowRollPreservesGravityAndPredictionConsistency) {
	for (int fast = 0; fast < 2; fast++) {
		FLT gain, phase, acceleration, position, prediction;
		ASSERT_EQ(roll_motion(fast ? 2 : .5, &gain, &phase, &acceleration, &position, &prediction), 0);
		fprintf(stderr,
				"Gyro control roll %.1fHz: gain %.6f, phase %.6fs, acceleration %.6fm/s2, position %.6fmm, prediction "
				"%.6fmm\n",
				fast ? 2. : .5, gain, phase, acceleration, position * 1000, prediction * 1000);
		ASSERT_EQ((fabs(gain - 1) < .05), true);
		ASSERT_EQ((fabs(phase) < .004), true);
		ASSERT_EQ((acceleration < .01), true);
		ASSERT_EQ((position < .0001), true);
		ASSERT_EQ((prediction < .00015), true);
	}
	return 0;
}
