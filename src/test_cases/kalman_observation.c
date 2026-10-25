#include "../survive_config.h"
#include "../survive_kalman_tracker.h"
#include "test_case.h"

static int hemisphere_equivalence(bool error_state, bool with_covariance) {
	FLT states[2][16] = {{0}}, covariances[2][16 * 16] = {{0}};
	int dimensions = 0;
	for (int sign = 0; sign < 2; sign++) {
		SurviveContext ctx = {0};
		config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
		ctx.temporary_config_values = &temporary;
		survive_configi(&ctx, "kalman-minimize-state-space", SC_SET | SC_OVERRIDE, 0);
		survive_configi(&ctx, "kalman-use-error-space", SC_SET | SC_OVERRIDE, error_state);
		survive_configi(&ctx, "kalman-align-observation-quaternion", SC_SET | SC_OVERRIDE, 1);
		SurviveObject so = {.ctx = &ctx, .timebase_hz = 48000000};
		SurviveKalmanTracker tracker;
		survive_kalman_tracker_init(&tracker, &so);
		tracker.model.t = 1;
		cn_set_zero(&tracker.model.P);
		for (int i = 0; i < tracker.model.P.rows; i++)
			cnMatrixSet(&tracker.model.P, i, i, .001);
		SurvivePose observation = {.Pos = {.01, -.02, .03}};
		LinmathVec3d axis = {.03, -.04, .05};
		quatfromaxisanglemag(observation.Rot, axis);
		FLT values[7 * 7] = {0}, original[7 * 7];
		CnMat R = cnMat(7, 7, values);
		for (int i = 0; i < 7; i++)
			cnMatrixSet(&R, i, i, .0001);
		cnMatrixSet(&R, 0, 5, .00003);
		cnMatrixSet(&R, 5, 0, .00003);
		cnMatrixSet(&R, 1, 6, -.00002);
		cnMatrixSet(&R, 6, 1, -.00002);
		if (sign) {
			for (int i = 0; i < 4; i++) observation.Rot[i] *= -1;
			for (int i = 0; i < 7; i++)
				for (int j = 0; j < 7; j++)
					if ((i < 3) != (j < 3))
						cnMatrixSet(&R, i, j, -cnMatrixGet(&R, i, j));
		}
		memcpy(original, values, sizeof(values));
		PoserData pd = {.pt = POSERDATA_LIGHT_GEN2, .timecode = 48000000};
		survive_kalman_tracker_integrate_observation(&pd, &tracker, &observation, with_covariance ? &R : NULL);
		memcpy(states[sign], &tracker.state, sizeof(states[sign]));
		dimensions = tracker.model.P.rows;
		for (int i = 0; i < dimensions; i++)
			for (int j = 0; j < dimensions; j++)
				covariances[sign][i * dimensions + j] = cnMatrixGet(&tracker.model.P, i, j);
		ASSERT_DOUBLE_ARRAY_EQ(49, values, original);
		survive_kalman_tracker_free(&tracker);
		for (int i = 0; i < temporary.used_entries; i++) {
			ASSERT_EQ(temporary.config_entries[i].update_list, NULL);
			free(temporary.config_entries[i].tag);
			free(temporary.config_entries[i].data);
		}
		free(temporary.config_entries);
		OGDeleteMutex(temporary.write_lock);
	}
	ASSERT_DOUBLE_ARRAY_EQ(16, states[0], states[1]);
	ASSERT_DOUBLE_ARRAY_EQ(dimensions * dimensions, covariances[0], covariances[1]);
	return 0;
}

TEST(KalmanObservation, AntipodalQuaternionWithCrossCovariance) {
	return hemisphere_equivalence(true, true);
}
TEST(KalmanObservation, AntipodalQuaternionWithoutCrossCovariance) {
	return hemisphere_equivalence(true, false);
}
TEST(KalmanObservation, AntipodalQuaternionWithoutErrorState) {
	return hemisphere_equivalence(false, true);
}
