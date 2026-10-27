#include "../survive_config.h"
#include "../survive_kalman_lighthouses.h"
#include "test_case.h"

static int lighthouse_hemisphere_equivalence(bool correlated) {
	for (int prior_sign = 0; prior_sign < 2; prior_sign++) {
		for (int stale_state_sign = 0; stale_state_sign < 2; stale_state_sign++) {
			SurvivePose states[2];
			FLT covariances[2][36];
			for (int measurement_sign = 0; measurement_sign < 2; measurement_sign++) {
				SurviveContext ctx = {.log_level = -1, .activeLighthouses = 1};
				config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
				ctx.temporary_config_values = &temporary;
				SurviveKalmanLighthouse *tracker = calloc(1, sizeof(*tracker));
				survive_kalman_lighthouse_init(tracker, &ctx, 0);
				ctx.bsd[0].tracker = tracker;
				SurvivePose prior = {.Pos = {1.3, -.6, 2.1}};
				LinmathVec3d prior_angle = {.7, -.3, .4};
				quatfromaxisanglemag(prior.Rot, prior_angle);
				if (prior_sign) for (int i = 0; i < 4; i++) prior.Rot[i] *= -1;
				tracker->state.Lighthouse = prior;
				tracker->push_state = tracker->state;
				// The recipient restores push_state before fusing its observation.
				// A stale, opposite state must not choose the measurement hemisphere.
				if (stale_state_sign) for (int i = 0; i < 4; i++) tracker->state.Lighthouse.Rot[i] *= -1;
				cn_set_zero(&tracker->model.P);
				cn_set_zero(&tracker->push_cov);
				cn_set_diag_val(&tracker->model.P, .002);
				cn_set_diag_val(&tracker->push_cov, .002);
				ctx.bsd[0].Pose = prior;
				ctx.bsd[0].PositionSet = 1;
				SurvivePose measured = prior;
				measured.Pos[0] += .012;
				measured.Pos[1] -= .008;
				LinmathVec3d angle = {.025, -.017, .021};
				LinmathQuat delta;
				quatfromaxisanglemag(delta, angle);
				quatrotateabout(measured.Rot, prior.Rot, delta);
				FLT values[49] = {0}, original[49];
				CnMat R = cnMat(7, 7, values);
				cn_set_diag_val(&R, .0001);
				if (correlated) {
					cnMatrixSet(&R, 0, 4, 2e-5); cnMatrixSet(&R, 4, 0, 2e-5);
					cnMatrixSet(&R, 1, 6, -3e-5); cnMatrixSet(&R, 6, 1, -3e-5);
				}
				if (measurement_sign) {
					for (int i = 0; i < 4; i++) measured.Rot[i] *= -1;
					for (int i = 0; i < 7; i++)
						for (int j = 0; j < 7; j++)
							if ((i < 3) != (j < 3)) cnMatrixSet(&R, i, j, -cnMatrixGet(&R, i, j));
				}
				memcpy(original, values, sizeof(values));
				survive_kalman_lighthouse_integrate_observation(tracker, &measured, &R);
				ASSERT_EQ(memcmp(original, values, sizeof(values)), 0);
				states[measurement_sign] = tracker->state.Lighthouse;
				for (int i = 0; i < 6; i++)
					for (int j = 0; j < 6; j++) covariances[measurement_sign][6 * i + j] = cnMatrixGet(&tracker->model.P, i, j);
				survive_kalman_lighthouse_free(tracker);
				for (int i = 0; i < temporary.used_entries; i++) {
					ASSERT_EQ(temporary.config_entries[i].update_list, NULL);
					free(temporary.config_entries[i].tag);
					free(temporary.config_entries[i].data);
				}
				free(temporary.config_entries);
				OGDeleteMutex(temporary.write_lock);
			}
			FLT tolerance = sizeof(FLT) == sizeof(float) ? 1e-5 : 1e-10;
			for (int i = 0; i < 7; i++) ASSERT_GT(tolerance, fabs(((FLT *)&states[0])[i] - ((FLT *)&states[1])[i]));
			for (int i = 0; i < 36; i++) ASSERT_GT(tolerance, fabs(covariances[0][i] - covariances[1][i]));
		}
	}
	return 0;
}

TEST(LighthouseObservation, EquivalentQuaternionSigns) { return lighthouse_hemisphere_equivalence(false); }
TEST(LighthouseObservation, EquivalentSignsWithPositionRotationCorrelation) { return lighthouse_hemisphere_equivalence(true); }
