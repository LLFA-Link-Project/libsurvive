#include "../survive_config.h"
#include "../survive_kalman_tracker.h"
#include "../generated/kalman_kinematics.gen.h"
#include "../generated/lighthouse_model.gen.h"
#include <survive_reproject_gen2.h>
#include "test_case.h"

TEST(KalmanObservation, FiniteErrorCoordinatesInvertTheirRetraction) {
	SurviveContext ctx = {0};
	config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
	ctx.temporary_config_values = &temporary;
	survive_configi(&ctx, "kalman-minimize-state-space", SC_SET | SC_OVERRIDE, 0);
	SurviveObject so = {.ctx = &ctx, .timebase_hz = 48000000};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	cnkalman_state_t *models[] = {&tracker.model, &tracker.imu_bias_model};
	for (int model = 0; model < 2; model++) {
		cnkalman_state_t *k = models[model];
		FLT prior_values[32] = {0}, next_values[32] = {0}, error_values[32] = {0}, recovered_values[32] = {0};
		CnMat prior = cnMat(k->state_cnt, 1, prior_values), next = cnMat(k->state_cnt, 1, next_values);
		CnMat error = cnMat(k->error_state_size, 1, error_values), recovered = cnMat(k->error_state_size, 1, recovered_values);
		memcpy(prior_values, cn_as_const_vector(&k->state), k->state_cnt * sizeof(FLT));
		LinmathVec3d axis = {.7, -.4, .5};
		quatfromaxisanglemag(prior_values + 3, axis);
		for (int sign = 0; sign < 2; sign++) {
			for (int i = 0; i < k->error_state_size; i++) error_values[i] = .003 * (i + 1);
			error_values[3] = .18; error_values[4] = -.11; error_values[5] = .14;
			if (sign) for (int i = 3; i < 7; i++) prior_values[i] *= -1;
			k->Update_fn(k->user, &prior, &error, &next, NULL);
			k->ErrorState_fn(k->user, &prior, &next, &recovered, NULL);
			for (int i = 0; i < k->error_state_size; i++) ASSERT_GT(1e-10, fabs(recovered_values[i] - error_values[i]));
			FLT update_values[32 * 32] = {0}, inverse_values[32 * 32] = {0};
			CnMat update = cnMat(k->state_cnt, k->error_state_size, update_values);
			CnMat inverse = cnMat(k->error_state_size, k->state_cnt, inverse_values);
			k->Update_fn(k->user, &prior, &error, NULL, &update);
			k->ErrorState_fn(k->user, &prior, &next, NULL, &inverse);
			const FLT epsilon = 1e-6;
			for (int col = 0; col < k->error_state_size; col++) {
				FLT plus_values[32] = {0}, minus_values[32] = {0};
				CnMat plus = cnMat(k->state_cnt, 1, plus_values), minus = cnMat(k->state_cnt, 1, minus_values);
				error_values[col] += epsilon;
				k->Update_fn(k->user, &prior, &error, &plus, NULL);
				error_values[col] -= 2 * epsilon;
				k->Update_fn(k->user, &prior, &error, &minus, NULL);
				error_values[col] += epsilon;
				for (int row = 0; row < k->state_cnt; row++)
					ASSERT_GT(5e-7, fabs((plus_values[row] - minus_values[row]) / (2 * epsilon) - cnMatrixGet(&update, row, col)));
			}
			for (int col = 0; col < k->state_cnt; col++) {
				FLT plus_values[32] = {0}, minus_values[32] = {0};
				CnMat plus = cnMat(k->error_state_size, 1, plus_values), minus = cnMat(k->error_state_size, 1, minus_values);
				next_values[col] += epsilon;
				k->ErrorState_fn(k->user, &prior, &next, &plus, NULL);
				next_values[col] -= 2 * epsilon;
				k->ErrorState_fn(k->user, &prior, &next, &minus, NULL);
				next_values[col] += epsilon;
				for (int row = 0; row < k->error_state_size; row++)
					ASSERT_GT(5e-7, fabs((plus_values[row] - minus_values[row]) / (2 * epsilon) - cnMatrixGet(&inverse, row, col)));
			}
		}
	}
	survive_kalman_tracker_free(&tracker);
	for (int i = 0; i < temporary.used_entries; i++) {
		free(temporary.config_entries[i].tag);free(temporary.config_entries[i].data);
	}
	free(temporary.config_entries);OGDeleteMutex(temporary.write_lock);
	return 0;
}

static int hemisphere_equivalence(bool error_state, bool with_covariance, bool local_rotation) {
	FLT states[2][16] = {{0}}, covariances[2][16 * 16] = {{0}};
	int dimensions = 0;
	for (int sign = 0; sign < 2; sign++) {
		SurviveContext ctx = {0};
		config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
		ctx.temporary_config_values = &temporary;
		survive_configi(&ctx, "kalman-minimize-state-space", SC_SET | SC_OVERRIDE, 0);
		survive_configi(&ctx, "kalman-use-error-space", SC_SET | SC_OVERRIDE, error_state);
		survive_configi(&ctx, "kalman-align-observation-quaternion", SC_SET | SC_OVERRIDE, 1);
		survive_configi(&ctx, "kalman-local-rotation-observation", SC_SET | SC_OVERRIDE, local_rotation);
		SurviveObject so = {.ctx = &ctx, .timebase_hz = 48000000};
		SurviveKalmanTracker tracker;
		survive_kalman_tracker_init(&tracker, &so);
		tracker.model.t = 1;
		if (local_rotation) {
			// Exercise a nonidentity orientation near the absolute axis-angle
			// branch cut, with noncommuting optical and prior rotations.
			LinmathVec3d prior_rotation = {1.3, -.8, 2.6};
			quatfromaxisanglemag(tracker.state.Pose.Rot, prior_rotation);
			tracker.obs_model.meas_jacobian_mode = cnkalman_jacobian_mode_debug;
		}
		cn_set_zero(&tracker.model.P);
		for (int i = 0; i < tracker.model.P.rows; i++)
			cnMatrixSet(&tracker.model.P, i, i, .001);
		SurvivePose observation = {.Pos = {.01, -.02, .03}};
		LinmathVec3d axis = {.03, -.04, .05};
		quatfromaxisanglemag(observation.Rot, axis);
		if (local_rotation)
			quatrotateabout(observation.Rot, tracker.state.Pose.Rot, observation.Rot);
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
		if (local_rotation) {
			ASSERT_EQ((tracker.obs_model.numeric_calcs > 0), true);
			ASSERT_EQ(tracker.obs_model.numeric_misses, 0);
		}
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
	return hemisphere_equivalence(true, true, false);
}
TEST(KalmanObservation, AntipodalQuaternionWithoutCrossCovariance) {
	return hemisphere_equivalence(true, false, false);
}
TEST(KalmanObservation, AntipodalQuaternionWithoutErrorState) {
	return hemisphere_equivalence(false, true, false);
}

TEST(KalmanObservation, LocalRotationAntipodalQuaternionWithCrossCovariance) {
	return hemisphere_equivalence(true, true, true);
}
TEST(KalmanObservation, LocalRotationAntipodalQuaternionWithoutCrossCovariance) {
	return hemisphere_equivalence(true, false, true);
}
TEST(KalmanObservation, LocalRotationAntipodalQuaternionWithoutErrorState) {
	return hemisphere_equivalence(false, true, true);
}

TEST(KalmanObservation, LocalRotationUpdateIsIndependentOfWorldOrientation) {
	SurvivePose reference = {0};
	for (int frame = 0; frame < 4; frame++) {
		SurviveContext ctx = {0};
		config_group temporary = {.write_lock = OGCreateMutex(), .ctx = &ctx};
		ctx.temporary_config_values = &temporary;
		survive_configi(&ctx, "kalman-minimize-state-space", SC_SET | SC_OVERRIDE, 0);
		survive_configi(&ctx, "kalman-local-rotation-observation", SC_SET | SC_OVERRIDE, 1);
		SurviveObject so = {.ctx = &ctx, .timebase_hz = 48000000};
		SurviveKalmanTracker tracker;
		survive_kalman_tracker_init(&tracker, &so);
		tracker.model.t = 1;
		cn_set_zero(&tracker.model.P);
		for (int i = 0; i < tracker.model.P.rows; i++)
			cnMatrixSet(&tracker.model.P, i, i, .001);
		LinmathQuat world;
		LinmathVec3d world_rotation = {frame * .7, frame * -.5, frame * .6};
		quatfromaxisanglemag(world, world_rotation);
		quatcopy(tracker.state.Pose.Rot, world);
		SurvivePose observation = {.Pos = {.01, -.02, .03}};
		LinmathVec3d local_rotation = {.13, -.07, .1};
		quatfromaxisanglemag(observation.Rot, local_rotation);
		quatrotateabout(observation.Rot, world, observation.Rot);
		quatrotatevector(observation.Pos, world, observation.Pos);
		PoserData pd = {.pt = POSERDATA_LIGHT_GEN2, .timecode = 48000000};
		survive_kalman_tracker_integrate_observation(&pd, &tracker, &observation, NULL);
		LinmathQuat inverse;
		quatgetconjugate(inverse, world);
		SurvivePose result = tracker.state.Pose;
		quatrotatevector(result.Pos, inverse, result.Pos);
		quatrotateabout(result.Rot, inverse, result.Rot);
		if (frame == 0) {
			reference = result;
		} else {
			ASSERT_DOUBLE_ARRAY_EQ(3, result.Pos, reference.Pos);
			ASSERT_QUAT_EQ(result.Rot, reference.Rot);
		}
		ASSERT_EQ(isfinite(tracker.stats.obs_total_error), true);
		survive_kalman_tracker_free(&tracker);
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

TEST(KalmanObservation, LocalRotationRejectsInvalidPoseBeforeChangingStateOrTime) {
	SurviveObject so = {.timebase_hz = 48000000};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	tracker.model.t = 1;
	SurviveKalmanModel original = tracker.state;
	for (int invalid = 0; invalid < 3; invalid++) {
		SurvivePose observation = LinmathPose_Identity;
		if (invalid == 0) observation.Rot[0] = 0;
		if (invalid == 1) observation.Rot[1] = NAN;
		if (invalid == 2) observation.Pos[0] = INFINITY;
		PoserData pd = {.pt = POSERDATA_LIGHT_GEN2, .timecode = 96000000};
		survive_kalman_tracker_integrate_observation(&pd, &tracker, &observation, NULL);
		ASSERT_DOUBLE_ARRAY_EQ(3, tracker.state.Pose.Pos, original.Pose.Pos);
		ASSERT_QUAT_EQ(tracker.state.Pose.Rot, original.Pose.Rot);
		ASSERT_EQ(tracker.stats.obs_count, 0);
		ASSERT_DOUBLE_EQ(tracker.model.t, 1.);
		ASSERT_DOUBLE_EQ(tracker.last_light_time, 0.);
		ASSERT_EQ(cn_is_finite(&tracker.model.P), true);
	}
	survive_kalman_tracker_free(&tracker);
	return 0;
}

TEST(KalmanObservation, OlderLightMeasurementsFollowConstantAcceleration) {
	SurviveKalmanModel state = {.Pose = {.Pos = {.3, -.2, 1.1}, .Rot = {.9, .1, -.2, .3}},
		.Velocity = {.Pos = {.02, -.03, .01}, .AxisAngleRot = {.01, -.02, .03}}, .Acc = {.4, -.2, .1},
		.IMUBias = {.AccScale = {1, 1, 1}, .IMUCorrection = {1}}};
	quatnormalize(state.Pose.Rot, state.Pose.Rot);
	SurvivePose camera = {.Pos = {.1, .2, -4}, .Rot = {1}};
	BaseStationCal calibration[2] = {{.phase = .001}, {.phase = -.002}};
	FLT point[] = {.03, -.04, .02};
	typedef FLT (*measurement_fn)(FLT, const SurviveKalmanModel *, const FLT *, const SurvivePose *, const BaseStationCal *);
	typedef FLT (*error_fn)(FLT, const SurviveKalmanModel *, const SurviveKalmanErrorModel *, const FLT *, const SurvivePose *, const BaseStationCal *);
	typedef void (*jac_fn)(CnMat *, FLT, const SurviveKalmanModel *, const FLT *, const SurvivePose *, const BaseStationCal *);
	typedef void (*error_jac_fn)(CnMat *, FLT, const SurviveKalmanModel *, const SurviveKalmanErrorModel *, const FLT *, const SurvivePose *, const BaseStationCal *);
	typedef FLT (*joint_fn)(FLT, const SurviveJointKalmanModel *, const SurviveJointKalmanErrorModel *, const FLT *);
	measurement_fn measurement[2][2] = {
		{SurviveKalmanModel_LightMeas_x_gen1, SurviveKalmanModel_LightMeas_y_gen1},
		{SurviveKalmanModel_LightMeas_x_gen2, SurviveKalmanModel_LightMeas_y_gen2}};
	error_fn error[2][2] = {
		{SurviveKalmanErrorModel_LightMeas_x_gen1, SurviveKalmanErrorModel_LightMeas_y_gen1},
		{SurviveKalmanErrorModel_LightMeas_x_gen2, SurviveKalmanErrorModel_LightMeas_y_gen2}};
	jac_fn jac[2][2] = {
		{SurviveKalmanModel_LightMeas_x_gen1_jac_x0, SurviveKalmanModel_LightMeas_y_gen1_jac_x0},
		{SurviveKalmanModel_LightMeas_x_gen2_jac_x0, SurviveKalmanModel_LightMeas_y_gen2_jac_x0}};
	error_jac_fn error_jac[2][2] = {
		{SurviveKalmanErrorModel_LightMeas_x_gen1_jac_error_model, SurviveKalmanErrorModel_LightMeas_y_gen1_jac_error_model},
		{SurviveKalmanErrorModel_LightMeas_x_gen2_jac_error_model, SurviveKalmanErrorModel_LightMeas_y_gen2_jac_error_model}};
	joint_fn joint[2][2] = {
		{SurviveJointKalmanErrorModel_LightMeas_x_gen1, SurviveJointKalmanErrorModel_LightMeas_y_gen1},
		{SurviveJointKalmanErrorModel_LightMeas_x_gen2, SurviveJointKalmanErrorModel_LightMeas_y_gen2}};
	SurviveKalmanErrorModel zero = {0};
	SurviveJointKalmanErrorModel joint_zero = {0};
	SurviveJointKalmanModel joint_state = {.Object = state, .Lighthouse = InvertPoseRtn(&camera),
		.BSD0 = calibration[0], .BSD1 = calibration[1]};
	FLT intervals[] = {-.04, -.003, .003, .04};
	for (int n = 0; n < sizeof(intervals) / sizeof(intervals[0]); n++) {
		FLT dt = intervals[n];
		SurvivePose physical = state.Pose;
		for (int axis = 0; axis < 3; axis++)
			physical.Pos[axis] += state.Velocity.Pos[axis] * dt + .5 * state.Acc[axis] * dt * dt;
		survive_apply_ang_velocity(physical.Rot, state.Velocity.AxisAngleRot, dt, state.Pose.Rot);
		for (int gen = 0; gen < 2; gen++) {
			FLT expected[2];
			(gen ? survive_reproject_full_gen2 : survive_reproject_full)(calibration, &camera, &physical, point, expected);
			for (int axis = 0; axis < 2; axis++) {
				ASSERT_EQ((fabs(measurement[gen][axis](dt, &state, point, &camera, &calibration[axis]) - expected[axis]) < 1e-10), true);
				ASSERT_EQ((fabs(error[gen][axis](dt, &state, &zero, point, &camera, &calibration[axis]) - expected[axis]) < 1e-10), true);
				ASSERT_EQ((fabs(joint[gen][axis](dt, &joint_state, &joint_zero, point) - expected[axis]) < 1e-10), true);
				CN_CREATE_STACK_MAT(H, 1, sizeof(state) / sizeof(FLT));
				CN_CREATE_STACK_MAT(He, 1, sizeof(zero) / sizeof(FLT));
				jac[gen][axis](&H, dt, &state, point, &camera, &calibration[axis]);
				error_jac[gen][axis](&He, dt, &state, &zero, point, &camera, &calibration[axis]);
				for (int acc = 0; acc < 3; acc++) {
					SurviveKalmanModel plus = state, minus = state;
					plus.Acc[acc] += .001;
					minus.Acc[acc] -= .001;
					FLT numeric = (measurement[gen][axis](dt, &plus, point, &camera, &calibration[axis]) -
						measurement[gen][axis](dt, &minus, point, &camera, &calibration[axis])) / .002;
					ASSERT_EQ((fabs(numeric - cnMatrixGet(&H, 0, offsetof(SurviveKalmanModel, Acc) / sizeof(FLT) + acc)) < 1e-8), true);
					ASSERT_EQ((fabs(numeric - cnMatrixGet(&He, 0, offsetof(SurviveKalmanErrorModel, Acc) / sizeof(FLT) + acc)) < 1e-8), true);
				}
			}
		}
	}
	return 0;
}

struct ManifoldMapProblem { SurviveKalmanModel prior; FLT precision[36], observed[9], variance[9]; };
static void manifold_map_measurement(const FLT *state, FLT *out) {
    memcpy(out,state,3*sizeof(FLT));
    const LinmathVec3d x={1,0,0},z={0,0,1};
    quatrotatevector(out+3,state+3,x);quatrotatevector(out+6,state+3,z);
}
static bool manifold_map_residual(void *user,const CnMat *Z,const CnMat *state,CnMat *y,CnMat *H) {
    FLT value[9];manifold_map_measurement(cn_as_const_vector(state),value);
    for (int i=0;i<9;i++)cnMatrixSet(y,i,0,cnMatrixGet(Z,i,0)-value[i]);
    if (H) {
        cn_set_zero(H);
        for (int col=0;col<7;col++) {
            FLT plus[16]={0},minus[16]={0},p[9],m[9];
            memcpy(plus,cn_as_const_vector(state),state->rows*sizeof(FLT));
            memcpy(minus,plus,state->rows*sizeof(FLT));
            plus[col]+=1e-6;minus[col]-=1e-6;manifold_map_measurement(plus,p);manifold_map_measurement(minus,m);
            for (int row=0;row<9;row++)cnMatrixSet(H,row,col,(p[row]-m[row])/2e-6);
        }
    }
    return true;
}
static void manifold_map_prior_error(const struct ManifoldMapProblem *p,const FLT *state,FLT *e) {
    for (int i=0;i<3;i++)e[i]=state[i]-p->prior.Pose.Pos[i];
    LinmathQuat inv,r;quatgetconjugate(inv,p->prior.Pose.Rot);quatrotateabout(r,inv,state+3);
    for (int i=0;i<3;i++)e[i+3]=2*r[i+1]/r[0];
}
static FLT manifold_map_cost(const struct ManifoldMapProblem *p,const FLT *state) {
    FLT e[6],value[9],v=0;manifold_map_prior_error(p,state,e);manifold_map_measurement(state,value);
    for (int i=0;i<6;i++)for(int j=0;j<6;j++)v+=.5*e[i]*p->precision[6*i+j]*e[j];
    for (int i=0;i<9;i++)v+=.5*(value[i]-p->observed[i])*(value[i]-p->observed[i])/p->variance[i];
    return v;
}
TEST(KalmanObservation, IteratedMapUsesThePriorAndPosteriorTangentFrames) {
    SurviveContext ctx={0};config_group temporary={.write_lock=OGCreateMutex(),.ctx=&ctx};ctx.temporary_config_values=&temporary;
    survive_configi(&ctx,"kalman-minimize-state-space",SC_SET|SC_OVERRIDE,0);
    SurviveObject so={.ctx=&ctx,.timebase_hz=48000000};SurviveKalmanTracker t;survive_kalman_tracker_init(&t,&so);
    t.model.t=1;const LinmathVec3d axis={.4,-.3,.6};quatfromaxisanglemag(t.state.Pose.Rot,axis);
    t.state.Pose.Pos[0]=.03;t.state.Pose.Pos[1]=-.02;t.state.Pose.Pos[2]=.04;
    struct ManifoldMapProblem p={.prior=t.state};
    FLT lower[36]={.08,0,0,0,0,0, 0,.06,0,0,0,0, 0,0,.09,0,0,0, .15,0,0,.6,0,0, 0,.1,0,0,.3,0, 0,0,-.2,0,0,.5};
    FLT covariance[36]={0};for(int i=0;i<6;i++)for(int j=0;j<6;j++)for(int k=0;k<6;k++)covariance[6*i+j]+=lower[6*i+k]*lower[6*j+k];
    CnMat P=cnMat(6,6,covariance),iP=cnMat(6,6,p.precision);cnInvert(&P,&iP,CN_INVERT_METHOD_LU);
    cn_set_zero(&t.model.P);for(int i=0;i<t.model.P.rows;i++)cnMatrixSet(&t.model.P,i,i,.01);
    for(int i=0;i<6;i++)for(int j=0;j<6;j++)cnMatrixSet(&t.model.P,i,j,covariance[6*i+j]);
    SurvivePose truth={.Pos={.12,-.08,.05}};const LinmathVec3d turn={.9,-.5,.7};LinmathQuat dq;quatfromaxisanglemag(dq,turn);
    quatrotateabout(truth.Rot,t.state.Pose.Rot,dq);manifold_map_measurement((const FLT *)&truth,p.observed);
    for(int i=0;i<9;i++)p.variance[i]=i<3 ? .005*.005 : .05*.05;
    CnMat Z=cnMat(9,1,p.observed),R=cnMat(9,1,p.variance);cnkalman_meas_model_t model;
    cnkalman_meas_model_init(&t.model,"map-contract",&model,manifold_map_residual);
    model.term_criteria.max_iterations=100;model.term_criteria.xtol=1e-12;model.term_criteria.mtol=1e-18;model.term_criteria.minimum_step=1e-7;
    struct cnkalman_update_extended_stats_t stats={0};
    FLT initial=manifold_map_cost(&p,(const FLT *)&t.state),result=cnkalman_meas_model_predict_update_stats(1,&model,&p,&Z,&R,&stats);
    FLT gradient[6]={0},information[36]={0},expected_values[36]={0},gradnorm=0,actualnorm=0,errornorm=0;
    FLT prior_jac[36]={0},meas_jac[54]={0};
    for(int col=0;col<6;col++) {
        FLT ev[15]={0},plus[16]={0},minus[16]={0},ep[6],em[6],mp[9],mm[9];
        CnMat E=cnMat(15,1,ev),xp=cnMat(16,1,plus),xm=cnMat(16,1,minus);
        ev[col]=1e-6;t.model.Update_fn(&p,&t.model.state,&E,&xp,NULL);
        ev[col]=-1e-6;t.model.Update_fn(&p,&t.model.state,&E,&xm,NULL);
        gradient[col]=(manifold_map_cost(&p,plus)-manifold_map_cost(&p,minus))/2e-6;gradnorm+=gradient[col]*gradient[col];
        manifold_map_prior_error(&p,plus,ep);manifold_map_prior_error(&p,minus,em);manifold_map_measurement(plus,mp);manifold_map_measurement(minus,mm);
        for(int i=0;i<6;i++)prior_jac[6*i+col]=(ep[i]-em[i])/2e-6;
        for(int i=0;i<9;i++)meas_jac[6*i+col]=(mp[i]-mm[i])/2e-6;
    }
    for(int i=0;i<6;i++)for(int j=0;j<6;j++) {
        for(int a=0;a<6;a++)for(int b=0;b<6;b++)information[6*i+j]+=prior_jac[6*a+i]*p.precision[6*a+b]*prior_jac[6*b+j];
        for(int a=0;a<9;a++)information[6*i+j]+=meas_jac[6*a+i]*meas_jac[6*a+j]/p.variance[a];
    }
    CnMat info=cnMat(6,6,information),expected=cnMat(6,6,expected_values);cnInvert(&info,&expected,CN_INVERT_METHOD_LU);
    for(int i=0;i<6;i++)for(int j=0;j<6;j++) {
        FLT x=cnMatrixGet(&t.model.P,i,j),d=x-expected_values[6*i+j];actualnorm+=expected_values[6*i+j]*expected_values[6*i+j];errornorm+=d*d;
    }
    FLT final=manifold_map_cost(&p,(const FLT *)&t.state),relative_covariance_error=sqrt(errornorm/actualnorm);
    survive_kalman_tracker_free(&t);for(int i=0;i<temporary.used_entries;i++){free(temporary.config_entries[i].tag);free(temporary.config_entries[i].data);}free(temporary.config_entries);OGDeleteMutex(temporary.write_lock);
    ASSERT_EQ(result >= 0, true);
    ASSERT_GT(initial, final);
    ASSERT_GT(1e-4, sqrt(gradnorm));
    ASSERT_GT(1e-5, relative_covariance_error);
    return 0;
}

static int iterated_linear_contract(bool reject) {
    FLT state[2] = {.2, -.3}, prior_P[4] = {.4, .1, .1, .3};
    FLT H_values[4] = {1, .3, -.2, 1}, observed[2] = {.1, .5}, variance[2] = {.05, .07};
    cnkalman_state_t k;
    cnkalman_state_init(&k, 2, NULL, NULL, NULL, state);
    k.t = 1;
    for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) cnMatrixSet(&k.P, i, j, prior_P[2*i+j]);
    CnMat H = cnMat(2, 2, H_values), Z = cnMat(2, 1, observed), R = cnMat(2, 1, variance);
    cnkalman_meas_model_t measurement;
    cnkalman_meas_model_init(&k, "linear contract", &measurement, NULL);
    measurement.term_criteria.max_iterations = 10;
    measurement.term_criteria.xtol = 1e-10;
    measurement.term_criteria.max_error = reject ? .1 : -1;
    struct cnkalman_update_extended_stats_t stats = {.total_stats = &measurement.stats};
    FLT result = cnkalman_meas_model_predict_update_stats(1, &measurement, &H, &Z, &R, &stats);
    if (reject) {
        ASSERT_EQ(result < 0, true);
        ASSERT_EQ(stats.stop_reason, cnkalman_update_extended_termination_reason_too_high_error);
        ASSERT_DOUBLE_EQ(state[0], .2);
        ASSERT_DOUBLE_EQ(state[1], -.3);
        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) ASSERT_DOUBLE_EQ(cnMatrixGet(&k.P, i, j), prior_P[2*i+j]);
    } else {
        // Closed-form Gaussian posterior for the fixed linear problem.
        FLT expected_state[2] = {.028499422810967734, .32483741556467366};
        FLT expected_P[4] = {.04339204279199482, -.004914678888596785, -.004914678888596813, .0512612437566434};
        ASSERT_EQ(result >= 0, true);
        for (int i = 0; i < 2; i++) ASSERT_GT(1e-10, fabs(state[i] - expected_state[i]));
        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) ASSERT_GT(1e-10, fabs(cnMatrixGet(&k.P, i, j) - expected_P[2*i+j]));
    }
    cnkalman_state_free(&k);
    return 0;
}

TEST(KalmanObservation, IteratedLinearMeasurementUsesTheProvidedJacobian) {
    return iterated_linear_contract(false);
}
TEST(KalmanObservation, IteratedErrorLimitRejectsBeforeChangingStateOrCovariance) {
    return iterated_linear_contract(true);
}

TEST(KalmanObservation, CorrectedCovarianceUsesTheNewNominalPose) {
	SurviveObject so = {.timebase_hz = 48000000};
	SurviveKalmanTracker tracker;
	survive_kalman_tracker_init(&tracker, &so);
	tracker.model.t = 1;
	ASSERT_EQ(tracker.model.state_cnt, 16);
	ASSERT_EQ(tracker.model.error_state_size, 15);
	tracker.state.Velocity.Pos[0] = .4;
	LinmathVec3d initial_rotation = {.7, -.3, .4};
	quatfromaxisanglemag(tracker.state.Pose.Rot, initial_rotation);
	cn_set_zero(&tracker.model.P);
	for (int i = 0; i < 15; i++) cnMatrixSet(&tracker.model.P, i, i, .02);
	cnMatrixSet(&tracker.model.P, 6, 6, .004);
	int indices[] = {0, 3, 4, 5};
	FLT cross[] = {.001, .004, -.003, .002};
	for (int i = 0; i < 4; i++) {
		cnMatrixSet(&tracker.model.P, indices[i], 6, cross[i]);
		cnMatrixSet(&tracker.model.P, 6, indices[i], cross[i]);
	}
	CN_CREATE_STACK_MAT(posterior, 15, 15);
	CN_CREATE_STACK_VEC(correction, 15);
	for (int row = 0; row < 15; row++) {
		FLT gain = cnMatrixGet(&tracker.model.P, row, 6) / .0044;
		cnMatrixSet(&correction, row, 0, -.4 * gain);
		for (int col = 0; col < 15; col++)
			cnMatrixSet(&posterior, row, col, cnMatrixGet(&tracker.model.P, row, col) -
				gain * cnMatrixGet(&tracker.model.P, 6, col));
	}
	SurviveKalmanModel prior = tracker.state;
	CnMat prior_state = cnVec(16, (FLT *)&prior);
	cnkalman_meas_model_t measurement;
	cnkalman_meas_model_init(&tracker.model, "known world velocity", &measurement, NULL);
	CN_CREATE_STACK_MAT(H, 1, 16);
	cnMatrixSet(&H, 0, 7, 1);
	FLT zero = 0, noise = .0004;
	CnMat Z = cnVec(1, &zero), R = cnVec(1, &noise);
	cnkalman_meas_model_predict_update(1, &measurement, &H, &Z, &R);
	ASSERT_DOUBLE_EQ(tracker.state.Velocity.Pos[0], .4 / 11);
	// The linear Gaussian posterior above is in the old error chart.
	// Re-express its physical perturbations about the corrected quaternion,
	// including the position/rotation cross terms, before comparing it to P.
	CN_CREATE_STACK_MAT(reset, 15, 15);
	cn_eye(&reset, 0);
	FLT epsilon = sizeof(FLT) == sizeof(float) ? .002 : 1e-6;
	for (int col = 3; col < 6; col++) {
		LinmathVec3d angles[2];
		FLT original = cnMatrixGet(&correction, col, 0);
		for (int side = 0; side < 2; side++) {
			cnMatrixSet(&correction, col, 0, original + (side ? epsilon : -epsilon));
			SurviveKalmanModel perturbed = {0};
			CnMat state = cnVec(16, (FLT *)&perturbed);
			tracker.model.Update_fn(NULL, &prior_state, &correction, &state, NULL);
			LinmathQuat inverse, relative;
			quatgetconjugate(inverse, tracker.state.Pose.Rot);
			quatrotateabout(relative, inverse, perturbed.Pose.Rot);
			scale3d(angles[side], relative + 1, 2);
		}
		cnMatrixSet(&correction, col, 0, original);
		for (int row = 0; row < 3; row++)
			cnMatrixSet(&reset, row + 3, col, (angles[1][row] - angles[0][row]) / (2 * epsilon));
	}
	CN_CREATE_STACK_MAT(expected, 15, 15);
	cn_ABAt_add(&expected, &reset, &posterior, NULL);
	for (int row = 0; row < 15; row++)
		for (int col = 0; col < 15; col++) {
			FLT difference = cnMatrixGet(&expected, row, col) - cnMatrixGet(&tracker.model.P, row, col);
			FLT tolerance = sizeof(FLT) == sizeof(float) ? 1e-5 : 1e-10;
			ASSERT_EQ((fabs(difference) < tolerance), true);
		}
	survive_kalman_tracker_free(&tracker);
	return 0;
}
