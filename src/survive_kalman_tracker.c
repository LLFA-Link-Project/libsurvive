#include "survive_kalman_tracker.h"
#include "generated/imu_model.gen.h"
#include "generated/kalman_kinematics.gen.h"
#include "linmath.h"
#include "math.h"
#include "survive_internal.h"

#include <cnkalman/kalman.h>

#include <assert.h>
#include <inttypes.h>
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif
#include <memory.h>
#include <stddef.h>
#include <survive_reproject.h>
#include <survive_reproject_gen2.h>

#include "generated/lighthouse_model.gen.h"

#include "generated/survive_reproject.aux.generated.h"
#include "survive_kalman_lighthouses.h"
#include "survive_recording.h"

#define SURVIVE_MODEL_MAX_STATE_CNT (sizeof(SurviveKalmanModel) / sizeof(FLT))

STATIC_CONFIG_ITEM(KALMAN_JOINT_IMU_BIAS, "kalman-joint-imu-bias", 'b',
	"Experimental: retain motion/IMU-bias cross covariance in one error-state filter", 0)

// clang-format off
STRUCT_CONFIG_SECTION(SurviveKalmanTracker)
	STRUCT_CONFIG_ITEM("light-error-threshold",  "Error limit to invalidate position",
					   -1., t->light_error_threshold)
	STRUCT_CONFIG_ITEM("min-report-time",
					   "Minimum report interval in s (-1 uses nominal IMU cadence with 1% clock tolerance)", -1., t->min_report_time)
	STRUCT_CONFIG_ITEM("report-covariance", "Report covariance matrix every n poses", -1, t->report_covariance_cnt);
	STRUCT_CONFIG_ITEM("report-sampled-cloud", "Show sample cloud from covariance", false, t->report_sampled_cloud);

	STRUCT_CONFIG_ITEM("report-ignore-start",  "Number of reports to ignore at startup", 0, t->report_ignore_start)
	STRUCT_CONFIG_ITEM("report-ignore-threshold",
					   "Minimum variance to report pose from the kalman filter", 1e-1, t->report_threshold_var)
	STRUCT_CONFIG_ITEM("light-ignore-threshold",
					   "Minimum variance to allow light data into the kalman filter", 1., t->light_threshold_var)
	STRUCT_CONFIG_ITEM("light-required-obs",
					   "Minimum observations to allow light data into the kalman filter", 16, t->light_required_obs)

    STRUCT_CONFIG_ITEM("light-max-error",  "Maximum error to integrate into lightcap", -1, t->lightcap_max_error)
    STRUCT_CONFIG_ITEM("kalman-light-variance",  "Variance of raw light sensor readings", 1e-2, t->light_var)
    STRUCT_CONFIG_ITEM("obs-cov-scale",  "Covariance matrix scaling for obs",
                       1, t->obs_cov_scale)
	STRUCT_CONFIG_ITEM("kalman-obs-axisangle",  "Process observation updates as axis angle poses", false, t->obs_axisangle_model)
	STRUCT_CONFIG_ITEM("kalman-align-observation-quaternion", "Experimental quaternion/covariance hemisphere alignment", false, t->obs_align_quaternion)
	STRUCT_CONFIG_ITEM("kalman-local-rotation-observation", "Fuse optical orientation errors in the local rotation tangent space", true, t->obs_local_rotation)
    STRUCT_CONFIG_ITEM("obs-pos-variance",  "Variance of position integration from light capture",
					   1e-6, t->obs_pos_var)
	STRUCT_CONFIG_ITEM("obs-rot-variance",  "Variance of rotation integration from light capture",
					   1e-7, t->obs_rot_var)

	STRUCT_CONFIG_ITEM("use-raw-obs",  "If true; the raw position from the solver is used and no filtering is applied", 0, t->use_raw_obs)

	STRUCT_CONFIG_ITEM("show-raw-obs", "Show position of raw poser output", 0, t->show_raw_obs)

	STRUCT_CONFIG_ITEM("light-error-for-lh-confidence",
					   "Whether or not to invalidate LH positions based on kalman errors", 0, t->use_error_for_lh_pos)

	STRUCT_CONFIG_ITEM("process-weight-jerk", "Legacy higher-order positional process noise", 0, t->params.process_weight_jerk)
	STRUCT_CONFIG_ITEM("process-weight-acc", "Acceleration process noise density (m^2/s^5)", 1, t->params.process_weight_acc)
	STRUCT_CONFIG_ITEM("process-weight-ang-vel", "Angular velocity variance per second", 1,
					   t->params.process_weight_ang_velocity)
	STRUCT_CONFIG_ITEM("process-weight-vel", "Velocity variance per second", 0, t->params.process_weight_vel)
	STRUCT_CONFIG_ITEM("process-weight-pos", "Position variance per second", 0, t->params.process_weight_pos)
	STRUCT_CONFIG_ITEM("process-weight-rot", "Rotation variance per second", 0, t->params.process_weight_rotation)
	STRUCT_CONFIG_ITEM("process-weight-acc-bias", "Acc bias variance per second", 1e-8, t->params.process_weight_acc_bias)
	STRUCT_CONFIG_ITEM("process-weight-gyro-bias", "Gyro bias variance per second", 1e-8, t->params.process_weight_gyro_bias)
	STRUCT_CONFIG_ITEM("kalman-minimize-state-space", "Minimize the state space", 1, t->minimize_state_space)
	STRUCT_CONFIG_ITEM("kalman-use-error-space", "Model using error state", true, t->use_error_state)
	STRUCT_CONFIG_ITEM("kalman-gyro-control", "Experimental: propagate with held gyro samples and their covariance (enable at initialization)", false, t->use_gyro_control)

	STRUCT_CONFIG_ITEM("kalman-joint-model-lightcap", "Ratio of confidence of LH over tracked object to use joint filter", -1, t->joint_lightcap_ratio)
	STRUCT_CONFIG_ITEM("kalman-joint-lightcap-minimum-sensors", "Minimum number of sensors for the joint model to run", 5, t->joint_min_sensor_cnt)
	STRUCT_CONFIG_ITEM("kalman-lightcap-minimum-sensors", "Minimum number of sensors for the lightcap model to run", 5, t->lightcap_min_sensor_cnt)

	STRUCT_CONFIG_ITEM("kalman-initial-imu-variance", "Initial variance in IMU frame", 0, t->params.initial_variance_imu_correction)
	STRUCT_CONFIG_ITEM("kalman-initial-acc-scale-variance", "Initial variance in IMU frame", 1e-6, t->params.initial_acc_scale_variance)
	STRUCT_CONFIG_ITEM("kalman-initial-acc-bias-variance", "Initial variance in IMU frame", 1e-6, t->params.initial_acc_bias_variance)
	STRUCT_CONFIG_ITEM("kalman-initial-gyro-variance", "Initial variance in gyro", 1e-6, t->params.initial_gyro_variance)

	STRUCT_CONFIG_ITEM("kalman-zvu-moving", "", -1, t->zvu_moving_var)
	STRUCT_CONFIG_ITEM("kalman-zvu-stationary", "", 1e-5, t->zvu_stationary_var)
	STRUCT_CONFIG_ITEM("kalman-zvu-stationary-ramp", "Seconds to introduce stationary constraints after detection", .5, t->zvu_stationary_ramp)
	STRUCT_CONFIG_ITEM("kalman-zvu-stationary-gyro-scale", "Corrected gyro rate that weakens stationary information (rad/s; 0 disables)", .02, t->zvu_stationary_gyro_scale)
	STRUCT_CONFIG_ITEM("kalman-zvu-stationary-velocity-sigma", "Allowed velocity during stationary ramp (m/s)", .1, t->zvu_stationary_velocity_sigma)
	STRUCT_CONFIG_ITEM("kalman-zvu-motion-guard-time", "Stable optical tracking before relaxing stationary assumptions (s)", .5, t->zvu_motion_guard_time)
	STRUCT_CONFIG_ITEM("kalman-zvu-motion-all-devices", "Apply gradual stationary constraints and gyro motion evidence to controllers and HMDs too", true, t->zvu_motion_all_devices)
	STRUCT_CONFIG_ITEM("kalman-zvu-optical-motion-window", "Optical motion evidence window for stationary constraints (s; 0 disables)", .6, t->optical_motion_window)
	STRUCT_CONFIG_ITEM("kalman-zvu-no-light", "", 1e-4, t->zvu_no_light_var)
	STRUCT_CONFIG_ITEM("kalman-zvu-no-light-time", "", .1, t->zvu_no_light_time)

	STRUCT_CONFIG_ITEM("kalman-noise-model", "0 is jerk acceleration model, 1 is simple model", 0, t->noise_model)

	STRUCT_CONFIG_ITEM("imu-acc-norm-penalty", "Penalty to IMU variance when reading high accel values", 0, t->acc_norm_penalty)
	STRUCT_CONFIG_ITEM("imu-acc-variance", "Variance of accelerometer", 1e-3, t->acc_var)
	STRUCT_CONFIG_ITEM("imu-gyro-variance", "Variance of gyroscope", 0.0000304617, t->gyro_var)

	STRUCT_CONFIG_ITEM("light-batch-size", "", 32, t->light_batchsize)
END_STRUCT_CONFIG_SECTION(SurviveKalmanTracker)

// clang-format off

MEAS_MDL_CONFIG(obj, obs, 10, -1)
MEAS_MDL_CONFIG(obj, imu, 0, -1)
MEAS_MDL_CONFIG(obj, zvu, 0, 0)
MEAS_MDL_CONFIG(obj, lightcap, 10, .1)
MEAS_MDL_CONFIG(joint, lightcap, 10, .1)

static inline void integrate_variance_tracker(SurviveKalmanTracker *tracker, struct variance_tracker* vtracker, const FLT* v, size_t size) {
	bool isStationary = SurviveSensorActivations_stationary_time(&tracker->so->activations) > 4800000;
	if(!isStationary) {
		variance_tracker_reset(vtracker);
	} else {
		variance_tracker_add(vtracker, v, size);
	}
}

FLT pid_update(struct pid_t* pid, FLT err, FLT dt) {
	FLT der = err - pid->err;
	pid->integration += err;
	FLT output = pid->Kp * err + (pid->Ki * pid->integration * dt) + (pid->Kd * der /dt);
	pid->err = err;
	return output;
}

static SurviveKalmanModel copy_model(const FLT *src, size_t state_size) {
	SurviveKalmanModel rtn = {
		.IMUBias = {
				.IMUCorrection = { 1. },
				.AccScale = { 1., 1, 1 }
		}
	};
	assert(state_size >= 7 && state_size * sizeof(FLT) <= sizeof(SurviveKalmanModel));
	memcpy(rtn.Pose.Pos, src, sizeof(FLT) * state_size);
    quatnormalize(rtn.Pose.Rot, rtn.Pose.Rot);
	quatnormalize(rtn.IMUBias.IMUCorrection, rtn.IMUBias.IMUCorrection);
	return rtn;
}
static SurviveJointKalmanModel copy_joint_model(const FLT *src, size_t state_size) {
	assert(state_size >= 14);
	SurviveJointKalmanModel rtn = {
		0
	};
	memcpy(&rtn, src, sizeof(FLT) * state_size);
	quatnormalize(rtn.Lighthouse.Rot, rtn.Lighthouse.Rot);
	return rtn;
}
static SurviveKalmanErrorModel copy_error_model(const CnMat* src) {
	SurviveKalmanErrorModel rtn = {
		0
	};
	assert(src->rows >= 7);
	memcpy(rtn.Pose.Pos, src->data, sizeof(FLT) * src->rows);
	return rtn;
}

static inline FLT survive_kalman_tracker_position_var2(SurviveKalmanTracker *tracker, FLT *var_diag, size_t cnt) {
	FLT _var_diag[SURVIVE_MODEL_MAX_STATE_CNT] = {0};
	if (var_diag == 0)
		var_diag = _var_diag;

	for (int i = 0; i < cnt; i++) {
		var_diag[i] = cnMatrixGet(&tracker->model.P, i, i);
	}

	// Output confidence describes motion. Including the bias marginal would
	// change the reporting gate merely by choosing a joint state layout.
	size_t motion_cnt = tracker->use_error_state ? offsetof(SurviveKalmanErrorModel, IMUBias) / sizeof(FLT) :
		offsetof(SurviveKalmanModel, IMUBias) / sizeof(FLT);
	return normnd2(var_diag, cnt < motion_cnt ? cnt : motion_cnt);
}

void kalman_model_normalize(void *user, struct CnMat* x) {
    SurviveKalmanModel state = copy_model(x->data, x->rows);
    quatnormalize(state.Pose.Rot, state.Pose.Rot);
    quatnormalize(state.IMUBias.IMUCorrection, state.IMUBias.IMUCorrection);
    memcpy(x->data, &state, sizeof(FLT) * x->rows);
}
static void normalize_model(SurviveKalmanTracker *pTracker) {
	/*
	FLT d = magnitude3d(pTracker->state.Pose.Rot + 1);
	pTracker->state.Pose.Rot[0] = 0;
	if(d < 1) {
		pTracker->state.Pose.Rot[0] = FLT_SQRT(1 - d * d);
	}
*/
	quatnormalize(pTracker->state.Pose.Rot, pTracker->state.Pose.Rot);
	quatnormalize(pTracker->state.IMUBias.IMUCorrection, pTracker->state.IMUBias.IMUCorrection);

	for (int i = 0; i < 3; i++) {
		pTracker->state.IMUBias.AccScale[i] = linmath_enforce_range(pTracker->state.IMUBias.AccScale[i], .95, 1.05);
		pTracker->state.IMUBias.GyroBias[i] = linmath_enforce_range(pTracker->state.IMUBias.GyroBias[i], -1e-1, 1e-1);
		pTracker->state.IMUBias.AccBias[i] = linmath_enforce_range(pTracker->state.IMUBias.AccBias[i], -1e-1, 1e-1);
	}
	for (int i = 0; i < 3; i++) {
		assert(isfinite(pTracker->state.Pose.Pos[i]));
	}
	for (int i = 0; i < 4; i++) {
		assert(isfinite(pTracker->state.Pose.Rot[i]));
	}
}

struct map_light_data_ctx {
	SurviveKalmanTracker *tracker;
};

typedef void (*SurviveJointKalmanModel_LightMeas_jac_x0_with_hx)(CnMat* Hx, CnMat* hx, const FLT dt, const SurviveJointKalmanModel * _x0, const FLT* sensor_pt, const BaseStationCal* bsc0);
typedef void (*SurviveJointKalmanErrorModel_LightMeas_jac_x0_with_hx)(CnMat* Hx, CnMat* hx, const FLT dt, const SurviveJointKalmanModel * _x0, const SurviveJointKalmanErrorModel* error_model, const FLT* sensor_pt);


const static SurviveJointKalmanErrorModel zero_error_joint_model = { 0 };
/*
SurviveJointKalmanModel_LightMeas_jac_x0_with_hx SurviveJointKalmanModel_LightMeas_jac_x0_with_hx_fns[2][2] = {
	{SurviveJointKalmanModel_LightMeas_x_gen1_jac_x0_with_hx, SurviveJointKalmanModel_LightMeas_y_gen1_jac_x0_with_hx},
	{SurviveJointKalmanModel_LightMeas_x_gen2_jac_x0_with_hx, SurviveJointKalmanModel_LightMeas_y_gen2_jac_x0_with_hx},
};
 */
SurviveJointKalmanErrorModel_LightMeas_jac_x0_with_hx SurviveJointKalmanErrorModel_LightMeas_jac_x0_with_hx_fns[2][2] = {
	{SurviveJointKalmanErrorModel_LightMeas_x_gen1_jac_error_model_with_hx, SurviveJointKalmanErrorModel_LightMeas_y_gen1_jac_error_model_with_hx},
	{SurviveJointKalmanErrorModel_LightMeas_x_gen2_jac_error_model_with_hx, SurviveJointKalmanErrorModel_LightMeas_y_gen2_jac_error_model_with_hx},
};

typedef void (*SurviveKalmanModel_LightMeas_jac_x0_with_hx)(CnMat* Hx, CnMat* hx, const FLT dt, const SurviveKalmanModel* _x0, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0);
typedef void (*SurviveKalmanErrorModel_LightMeas_jac_x0_with_hx)(CnMat* Hx, CnMat* hx, const FLT dt, const SurviveKalmanModel* _x0, const SurviveKalmanErrorModel* error_model, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0);

const static SurviveKalmanErrorModel zero_error_model = { 0 };
SurviveKalmanModel_LightMeas_jac_x0_with_hx SurviveKalmanModel_LightMeas_jac_x0_with_hx_fns[2][2] = {
	{SurviveKalmanModel_LightMeas_x_gen1_jac_x0_with_hx, SurviveKalmanModel_LightMeas_y_gen1_jac_x0_with_hx},
	{SurviveKalmanModel_LightMeas_x_gen2_jac_x0_with_hx, SurviveKalmanModel_LightMeas_y_gen2_jac_x0_with_hx},
};
SurviveKalmanErrorModel_LightMeas_jac_x0_with_hx SurviveKalmanErrorModel_LightMeas_jac_x0_with_hx_fns[2][2] = {
	{SurviveKalmanErrorModel_LightMeas_x_gen1_jac_error_model_with_hx, SurviveKalmanErrorModel_LightMeas_y_gen1_jac_error_model_with_hx},
	{SurviveKalmanErrorModel_LightMeas_x_gen2_jac_error_model_with_hx, SurviveKalmanErrorModel_LightMeas_y_gen2_jac_error_model_with_hx},
};

static bool map_joint_light_data(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
						   struct CnMat *H_k) {
	struct map_light_data_ctx *cbctx = (struct map_light_data_ctx *)user;
	const SurviveKalmanTracker *tracker = cbctx->tracker;
	SurviveObject *so = tracker->so;
	struct SurviveContext *ctx = tracker->so->ctx;

    struct SurviveJointKalmanModel s = copy_joint_model(cn_as_const_vector(x_t), x_t->rows);

	if(H_k) {
		cn_set_zero(H_k);
	}

	CN_CREATE_STACK_VEC(h_x, 1);
	FLT *Y = cn_as_vector(y);
	for (int i = 0; i < Z->rows; i++) {
		const LightInfo *info = &tracker->savedLight[tracker->savedLight_idx + i];
		int axis = info->axis;

		assert(ctx->bsd[info->lh].PositionSet);

		const FLT *pt = &so->sensor_locations[info->sensor_idx * 3];
		SurvivePose imu2trackref = so->imu2trackref;
		LinmathPoint3d ptInObj = { 0 };
		gen_scale_sensor_pt(ptInObj, pt, &imu2trackref, so->sensor_scale);

		FLT t = info->timecode / 48000000. - cbctx->tracker->model.t;
		t = 0;
		CnMat H_k_row = {0};
		if(H_k) {
			H_k_row = cnMatView(1, H_k->cols, H_k, i, 0);
		}
		if(cbctx->tracker->lightcap_model.error_state_model && cbctx->tracker->use_error_state) {
			//assert(H_k == 0 || (cbctx->tracker->model.error_state_size + 6) == H_k->cols);
			SurviveJointKalmanErrorModel_LightMeas_jac_x0_with_hx_fns[ctx->lh_version][info->axis](H_k ? &H_k_row : 0,
																							  y ? &h_x : 0, t, &s,
																								   &zero_error_joint_model,
																								   ptInObj);
		} else {
			assert(false);
			//assert(H_k == 0 || cbctx->tracker->model.state_cnt == H_k->cols);
			//SurviveJointKalmanModel_LightMeas_jac_x0_with_hx_fns[ctx->lh_version][info->axis](H_k ? &H_k_row : 0,
			//																			 y ? &h_x : 0, t, &s, ptInObj, &world2lh, &ctx->bsd[info->lh].fcal[axis]);
		}
		if(y) {
			Y[i] = cn_as_const_vector(Z)[i] - h_x.data[0];
			if(tracker->lightcap_max_error > 0) {
				Y[i] = linmath_enforce_range(Y[i], -tracker->lightcap_max_error, tracker->lightcap_max_error);
			}
			SV_DATA_LOG("h_light[%d][%d][%d]", h_x.data, 1, info->lh, info->axis, info->sensor_idx);
			SV_DATA_LOG("Y_light[%d][%d][%d]", Y, 1, info->lh, info->axis, info->sensor_idx);
		}
		SV_DATA_LOG("Z_light[%d][%d][%d]", &info->value, 1, info->lh, info->axis, info->sensor_idx);
	}

	if (H_k && !cn_is_finite(H_k))
		return false;

	survive_recording_write_matrix(tracker->so->ctx->recptr, tracker->so, 100, "light-y", y);

	return true;
}
/**
 * This function reuses the reproject functions to estimate what it thinks the lightcap angle should be based on x_t,
 * and uses that measurement to compare from the actual observed angle. These functions have jacobian functions that
 * correspond to them; see @survive_reproject.c and @survive_reproject_gen2.c
 */
static bool map_light_data(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
						   struct CnMat *H_k) {
	struct map_light_data_ctx *cbctx = (struct map_light_data_ctx *)user;

	SurviveKalmanModel s = copy_model(cn_as_const_vector(x_t), x_t->rows);

	const SurviveKalmanTracker *tracker = cbctx->tracker;

	SurviveObject *so = tracker->so;
	struct SurviveContext *ctx = tracker->so->ctx;
	const survive_reproject_model_t *mdl = survive_reproject_model(ctx);

	if(H_k) {
	    cn_set_zero(H_k);
	}
    SurvivePose obj2world = *(SurvivePose *)cn_as_const_vector(x_t);
    quatnormalize(obj2world.Rot, obj2world.Rot);

	CN_CREATE_STACK_VEC(h_x, 1);
	FLT *Y = cn_as_vector(y);
	for (int i = 0; i < Z->rows; i++) {
		const LightInfo *info = &tracker->savedLight[tracker->savedLight_idx + i];
		int axis = info->axis;

		assert(ctx->bsd[info->lh].PositionSet);

		const SurvivePose world2lh = InvertPoseRtn(survive_get_lighthouse_position(ctx, info->lh));

		const FLT *pt = &so->sensor_locations[info->sensor_idx * 3];
        SurvivePose imu2trackref = so->imu2trackref;
        LinmathPoint3d ptInObj;
        gen_scale_sensor_pt(ptInObj, pt, &imu2trackref, so->sensor_scale);

		FLT t = info->timecode / 48000000. - cbctx->tracker->model.t;
		t = 0;
		CnMat H_k_row = {0};
		if(H_k) {
			H_k_row = cnMatView(1, H_k->cols, H_k, i, 0);
		}
		if(cbctx->tracker->lightcap_model.error_state_model && cbctx->tracker->use_error_state) {
			assert(H_k == 0 || cbctx->tracker->model.error_state_size == H_k->cols);
			SurviveKalmanErrorModel_LightMeas_jac_x0_with_hx_fns[ctx->lh_version][info->axis](H_k ? &H_k_row : 0,
																							  y ? &h_x : 0, t, &s, &zero_error_model, ptInObj, &world2lh,
																							  survive_basestation_cal(ctx, info->lh, axis));
		} else {
			assert(H_k == 0 || cbctx->tracker->model.state_cnt == H_k->cols);
			SurviveKalmanModel_LightMeas_jac_x0_with_hx_fns[ctx->lh_version][info->axis](H_k ? &H_k_row : 0,
																						 y ? &h_x : 0, t, &s, ptInObj, &world2lh,
																						 survive_basestation_cal(ctx, info->lh, axis));
		}
		if(y) {
			Y[i] = cn_as_const_vector(Z)[i] - h_x.data[0];
			if(tracker->lightcap_max_error > 0) {
				Y[i] = linmath_enforce_range(Y[i], -tracker->lightcap_max_error, tracker->lightcap_max_error);
			}
			SV_DATA_LOG("h_light[%d][%d][%d]", h_x.data, 1, info->lh, info->axis, info->sensor_idx);
			SV_DATA_LOG("Y_light[%d][%d][%d]", Y, 1, info->lh, info->axis, info->sensor_idx);
		}
        SV_DATA_LOG("Z_light[%d][%d][%d]", &info->value, 1, info->lh, info->axis, info->sensor_idx);
	}

	if (H_k && !cn_is_finite(H_k))
		return false;

	survive_recording_write_matrix(tracker->so->ctx->recptr, tracker->so, 100, "light-y", y);

	return true;
}

int sort_by_lh_axis_sensor(const void * _info1, const void * _info2) {
	const LightInfo* info1 = _info1;
	const LightInfo* info2 = _info2;
	if(info1->lh != info2->lh)
		return (info1->lh > info2->lh) ? -1 : 1;
	return 0;
}


void survive_kalman_tracker_integrate_saved_light(SurviveKalmanTracker *tracker, PoserData *pd) {
	SurviveContext *ctx = tracker->so->ctx;
	FLT time = pd->timecode / (FLT)tracker->so->timebase_hz;
	if (tracker->use_raw_obs) {
		return;
	}

	// A single light cap measurement has an infinite amount of solutions along a plane; so it only helps if we are
	// already in a good place
	if (tracker->light_threshold_var > 0 &&
		survive_kalman_tracker_position_var2(tracker, 0, 7) > tracker->light_threshold_var) {
		return;
	}

	if (tracker->light_required_obs > tracker->stats.obs_count) {
		return;
	}

	if (tracker->light_var >= 0) {
		for (int i = 0; i < tracker->savedLight_idx; i++) {
			if (!ctx->bsd[tracker->savedLight[i].lh].PositionSet) {
				tracker->savedLight[i] = tracker->savedLight[tracker->savedLight_idx - 1];
				tracker->savedLight_idx--;
				i--;
			}
		}

        if (tracker->savedLight_idx == 0) {
            return;
        }

		bool useJointModel = tracker->joint_lightcap_ratio >= 0;
		if(useJointModel) {
			qsort(tracker->savedLight, tracker->savedLight_idx, sizeof(tracker->savedLight[0]), sort_by_lh_axis_sensor);
		}

		FLT rtn = 0;
		while(tracker->savedLight_idx > 0) {
			int lh = tracker->savedLight[tracker->savedLight_idx-1].lh;
			int cnt = 0;

			if(useJointModel) {
				while(tracker->savedLight_idx > 0) {
					cnt++;
					tracker->savedLight_idx--;
					if(tracker->savedLight_idx > 0 && tracker->savedLight[tracker->savedLight_idx - 1].lh != lh)
						break;
				}
				if (cnt < tracker->joint_min_sensor_cnt) {
					cnt += tracker->savedLight_idx;
					tracker->savedLight_idx = 0;
					useJointModel = false;
					tracker->stats.joint_model_dropped++;
				}
			} else {
				cnt = tracker->savedLight_idx;
				tracker->savedLight_idx = 0;
			}

			if(cnt < tracker->lightcap_min_sensor_cnt) {
				tracker->stats.lightcap_model_dropped++;
				return;
			}

			CN_CREATE_STACK_VEC(Z, cnt);
			for (int i = tracker->savedLight_idx; i < cnt + tracker->savedLight_idx; i++) {
				assert(useJointModel == false || tracker->savedLight[i].lh == lh);
				cnMatrixSet(&Z, i - tracker->savedLight_idx, 0, tracker->savedLight[i].value);
			}

			struct map_light_data_ctx cbctx = {
				.tracker = tracker,
			};

			SurviveObject *so = tracker->so;
			FLT light_var = tracker->light_var;

			SV_DATA_LOG("light_var", &light_var, 1);
			FLT light_vars[32] = {0};
			for (int i = 0; i < 32; i++)
				light_vars[i] = light_var;
			CnMat R = cnVec(Z.rows, light_vars);

			tracker->datalog_tag = "light_data";
			if(time < tracker->model.t)
				time = tracker->model.t;

			FLT obj_trace = cn_trace(&tracker->model.P);
			FLT lh_trace = cn_trace(&ctx->bsd[lh].tracker->model.P);
			FLT ratio = lh_trace / obj_trace;

			tracker->last_light_time = time;

			if(useJointModel && tracker->joint_lightcap_ratio < ratio) {
				tracker->joint_model.ks[0] = &ctx->bsd[lh].tracker->model;
				tracker->joint_model.ks[1] = &ctx->bsd[lh].tracker->bsd_model;
				rtn += cnkalman_meas_model_predict_update(time, &tracker->joint_model, &cbctx, &Z, &R);
				tracker->stats.joint_model_sensor_cnt_sum += cnt;
				survive_kalman_lighthouse_report(ctx->bsd[lh].tracker);
			} else {
				rtn += cnkalman_meas_model_predict_update(time, &tracker->lightcap_model, &cbctx, &Z, &R);
				tracker->stats.lightcap_model_sensor_cnt_sum += cnt;
			}
		}

		tracker->datalog_tag = 0;
		tracker->stats.lightcap_total_error += rtn;

		tracker->light_residuals_all *= .9;
		tracker->light_residuals_all += .1 * rtn;

		SurviveObject * so = tracker->so;
		SV_DATA_LOG("res_error_light_", &rtn, 1);
		SV_DATA_LOG("res_error_light_avg", &tracker->light_residuals_all, 1);
		tracker->stats.lightcap_count++;

		survive_kalman_tracker_report_state(pd, tracker);
	}
}

void survive_kalman_tracker_integrate_light(SurviveKalmanTracker *tracker, PoserDataLight *data) {
	bool isSync = data->hdr.pt == POSERDATA_SYNC || data->hdr.pt == POSERDATA_SYNC_GEN2;
	if (isSync) {
		survive_kalman_tracker_integrate_saved_light(tracker, &data->hdr);
		tracker->savedLight_idx = 0;
	} else {
		LightInfo *info = &tracker->savedLight[tracker->savedLight_idx++];

		info->lh = data->lh;
		info->value = data->angle;
		info->axis = PoserDataLight_axis(data);
		info->sensor_idx = data->sensor_id;
		info->timecode = data->hdr.timecode;

		integrate_variance_tracker(tracker, &tracker->light_variance[info->lh][info->sensor_idx][info->axis], &info->value, 1);
	}

	int batchtrigger = sizeof(tracker->savedLight) / sizeof(tracker->savedLight[0]);
	if (tracker->light_batchsize >= 0) {
		batchtrigger = tracker->light_batchsize;
	}
	if (tracker->savedLight_idx >= batchtrigger) {
		survive_kalman_tracker_integrate_saved_light(tracker, &data->hdr);
		tracker->savedLight_idx = 0;
	}
}

struct map_imu_data_ctx {
	bool use_gyro, use_accel;
	SurviveKalmanTracker *tracker;
};

SURVIVE_EXPORT void survive_kalman_tracker_correct_imu(SurviveKalmanTracker *tracker, LinmathVec3d out, const LinmathVec3d accel) {
    // IMUMeasurementModel predicts scale * acceleration + bias.
    for(int i = 0;i < 3;i++) {
        out[i] = (accel[i] - tracker->state.IMUBias.AccBias[i]) / tracker->state.IMUBias.AccScale[i];
    }
}
/**
 * The prediction for IMU given x_t is:
 *
 * [Position, Rotation, Velocity, Ang_Velocity, Acc, Gyro_Bias] = x_t
 *
 * acc_predict  = Rotation^-1 * (Acc/9.80665 + [0, 0, 1])
 * gyro_predict = Rotation^-1 * Ang_Velocity + Gyro_Bias
 *
 * The actual code for this is generated from tools/generate_math_functions/imu_functions.py. It isn't done in
 * C natively to allow for the jacobian code to be generated using symengine
 */
static bool imu_measurement_model(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
								  struct CnMat *H_k, bool accelerometer_only) {
	struct map_imu_data_ctx *fn_ctx = user;

	CN_CREATE_STACK_VEC(h_x, 6);
	SurviveKalmanTracker *tracker = (SurviveKalmanTracker *)user;

	SurviveKalmanModel s = copy_model(cn_as_const_vector(x_t), x_t->rows);
	IMUMeasurementModel(&h_x, &s);

    if(H_k) {
        assert(H_k->rows == (accelerometer_only ? 3 : 6));

		bool errorState = fn_ctx->tracker->imu_model.error_state_model;
		if(errorState) {
			IMUMeasurementErrorModel_jac_error_model(H_k, &s, &zero_error_model);
		} else {
			IMUMeasurementModel_jac_model(H_k, &s);
		}

    }

	subnd(cn_as_vector(y), cn_as_const_vector(Z), cn_as_const_vector(&h_x), Z->rows);

	if(fn_ctx) {
		SurviveKalmanTracker * tracker = fn_ctx->tracker;
		SurviveObject * so = fn_ctx->tracker->so;
		SurviveContext *ctx = so->ctx;
		survive_recording_write_matrix(tracker->so->ctx->recptr, tracker->so, 100, "imu-y", y);
		SV_VERBOSE(600, "X     " Point7_format, LINMATH_VEC7_EXPAND(cn_as_const_vector(x_t)))
		if (accelerometer_only) {
			SV_VERBOSE(600, "Z     " Point3_format, LINMATH_VEC3_EXPAND(cn_as_const_vector(Z)))
		} else {
			SV_VERBOSE(600, "Z     " Point6_format, LINMATH_VEC6_EXPAND(cn_as_const_vector(Z)))
		}

		if(so->ctx->datalogproc) {
			SV_DATA_LOG("imu_prediction", h_x.data, 6);

			LinmathVec3d up = {0, 0, 1};
			FLT q[5];
			LinmathVec3d imuWorld;
			quatrotatevector(imuWorld, tracker->state.Pose.Rot, Z->data);

			quatfrom2vectors(q, imuWorld, up);
			q[4] = norm3d(q + 1);
			quatrotateabout(q, q, tracker->state.Pose.Rot);
			SV_DATA_LOG("perfect_q", q, 5);

			LinmathVec3d perfect_acc;
			quatrotatevector(perfect_acc, q, Z->data);
			perfect_acc[2] -= 1;
			SV_DATA_LOG("perfect_acc", perfect_acc, 3);
		}
	}

	return true;
}

bool survive_kalman_tracker_imu_measurement_model(void *user, const CnMat *Z, const CnMat *state, CnMat *y, CnMat *H) {
	return imu_measurement_model(user, Z, state, y, H, false);
}

static bool accelerometer_measurement_model(void *user, const CnMat *Z, const CnMat *state, CnMat *y, CnMat *H) {
	return imu_measurement_model(user, Z, state, y, H, true);
}

static void tracker_datalog(const cnkalman_state_t* state, const char *desc, const FLT *v, size_t length) {
	SurviveKalmanTracker *tracker = state->datalog_user;
	SurviveObject * so = tracker->so;

	if(tracker->datalog_tag == 0)
		tracker->datalog_tag = "unknown";

	SV_DATA_LOG("%s_%s", v, length, desc, tracker->datalog_tag);
}
static SurviveIMUBiasModel copy_imu_bias_model(const struct CnMat *x0) {
	SurviveIMUBiasModel state = { .IMUCorrection = { 1 }, .AccScale = {1., 1, 1} };
	assert(x0->rows <= sizeof(SurviveIMUBiasModel) / sizeof(FLT));
	memcpy(&state, cn_as_const_vector(x0), x0->rows * sizeof(FLT));
	return state;
}

static void imu_error_state_fn(void *user, const struct CnMat *x0,
							   const struct CnMat *x1, struct CnMat *E,
							   struct CnMat *E_jac_x) {
	SurviveIMUBiasModel state0 = copy_imu_bias_model(x0);
	SurviveIMUBiasModel state1 = x1 ? copy_imu_bias_model(x1) : state0;
	if(E_jac_x) {
		SurviveIMUBiasModelToErrorModel_jac_x1(E_jac_x, &state1, &state0);
	}

	if(x1 && E) {
		SurviveIMUBiasErrorModel error_state = { 0 };
		SurviveIMUBiasModelToErrorModel(&error_state, &state1, &state0);
		memcpy(cn_as_vector(E), &error_state, sizeof(FLT) * E->rows);
	}
}

static void error_state_fn(void *user, const struct CnMat *x0,
						   const struct CnMat *x1, struct CnMat *E,
						   struct CnMat *E_jac_x) {
	SurviveKalmanModel state0 = copy_model(cn_as_const_vector(x0), x0->rows);
	SurviveKalmanModel state1 = x1 ? copy_model(cn_as_const_vector(x1), x1->rows) : state0;
	if(E_jac_x) {
		SurviveKalmanModelToErrorModel_jac_x1(E_jac_x, &state1, &state0);
	}

	if(x1 && E) {
		SurviveKalmanErrorModel error_state = { 0 };
		SurviveKalmanModelToErrorModel(&error_state, &state1, &state0);
		memcpy(cn_as_vector(E), &error_state, sizeof(FLT) * E->rows);
	}
}

static void imu_bias_update_fn(void *user, const struct CnMat *x0, const struct CnMat *E, struct CnMat *x1, struct CnMat* dX_wrt_error_state) {
	SurviveIMUBiasModel state = copy_imu_bias_model(x0);
	const FLT* x0v = cn_as_const_vector(x0);
	if(x1){
		SurviveIMUBiasModel _x1 = { 0 };
		SurviveIMUBiasErrorModel error_state = { 0 };
		memcpy(&error_state, cn_as_const_vector(E), E->rows * sizeof(FLT));
		SurviveIMUBiasModelAddErrorModel(&_x1, &state, &error_state);
		memcpy(cn_as_vector(x1), &_x1, sizeof(FLT) * x1->rows);
	}
	if(dX_wrt_error_state){
		SurviveIMUBiasErrorModel error_model = { 0 };
		if (E) memcpy(&error_model, cn_as_const_vector(E), E->rows * sizeof(FLT));
		SurviveIMUBiasModelAddErrorModel_jac_error_state(dX_wrt_error_state, &state, &error_model);
	}
}

static void state_update_fn(void *user, const struct CnMat *x0, const struct CnMat *E, struct CnMat *x1, struct CnMat* dX_wrt_error_state) {
	SurviveKalmanModel state = copy_model(cn_as_const_vector(x0), x0->rows);
	const FLT* x0v = cn_as_const_vector(x0);
	if(x1){
		SurviveKalmanModel _x1 = { 0 };
		SurviveKalmanErrorModel error_state = copy_error_model(E);
		SurviveKalmanModelAddErrorModel(&_x1, &state, &error_state);
		memcpy(cn_as_vector(x1), &_x1, sizeof(FLT) * x1->rows);
	}
	if(dX_wrt_error_state){
		SurviveKalmanErrorModel error_model = E ? copy_error_model(E) : (SurviveKalmanErrorModel){0};
		SurviveKalmanModel state = copy_model(cn_as_const_vector(x0), x0->rows);
		SurviveKalmanModelAddErrorModel_jac_error_state(dX_wrt_error_state, &state, &error_model);
	}
}
/*
static void error_state_fn(void *user, const struct CnMat *x0, struct CnMat *H) {
	cn_set_diag_val(H, 1);
}
static void state_update_fn(void *user, const struct CnMat *x0, struct CnMat *Ky, struct CnMat *x1) {
	cn_elementwise_add(x1, x0, Ky);
}
*/
static bool map_obs_data(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
								   struct CnMat *H_k) {
	SurviveKalmanTracker *tracker = (SurviveKalmanTracker *)user;
	if(y) {
		subnd(cn_as_vector(y), cn_as_const_vector(Z), cn_as_const_vector(x_t), 7);
		survive_recording_write_matrix(tracker->so->ctx->recptr, tracker->so, 100, "obs-y", y);
	}
	if(H_k) {
		bool errorState = tracker->use_error_state && tracker->obs_model.error_state_model;
		if(errorState) {
			state_update_fn(user, x_t, 0, 0, H_k);
		} else {
			cn_set_zero(H_k);
			cn_set_diag_val(H_k, 1);
		}
	}
	return true;
}
static bool map_obs_data_axisangle(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
								   struct CnMat *H_k) {
	SurviveKalmanTracker *tracker = (SurviveKalmanTracker *)user;
	const SurviveKalmanModel *x0 = (const SurviveKalmanModel *)cn_as_const_vector(x_t);

	SurviveAxisAnglePose yp = { 0 };
	SurviveAxisAnglePose predictedPose = *(const SurviveAxisAnglePose *)cn_as_const_vector(Z);
	SurviveObsErrorModelNoFlip(&yp, x0, (const SurviveAxisAnglePose *)cn_as_const_vector(Z));
	scalend((FLT *)&yp, (FLT *)&yp, -1, 6);

	FLT mag = normnd2(yp.AxisAngleRot, 3);
	bool hasFlip = mag > M_PI * M_PI;
	if(hasFlip) {
		mag = sqrt(mag);
		scalend(yp.AxisAngleRot, yp.AxisAngleRot, (mag - 2.*M_PI) / mag, 3);
	}
	assert(norm3d(yp.AxisAngleRot) < M_PI);

	if(y) {
		memcpy(cn_as_vector(y), &yp, y->rows * sizeof(FLT));
	}
    if(H_k) {
		bool errorState = tracker->use_error_state && tracker->obs_model.error_state_model;
		if(errorState) {
			SurviveKalmanErrorModel error_model = { 0 };
			(!hasFlip ?
			 SurviveObsErrorStateErrorModelNoFlip_jac_err :
			 SurviveObsErrorStateErrorModelFlip_jac_err)(H_k, x0, &error_model, &predictedPose);
		} else {
			(!hasFlip ? SurviveObsErrorModelNoFlip_jac_x0 :
			 SurviveObsErrorModelFlip_jac_x0)(H_k, x0, &predictedPose);
		}
    }
    return true;
}

typedef struct {
	SurviveKalmanTracker *tracker;
	const SurvivePose *pose;
} LocalRotationObservation;

// Log(q), with the shortest rotation and its derivative in quaternion coordinates.
// This is homogeneous in q: the radial quaternion direction carries no information.
static void rotation_log_jacobian(const LinmathQuat q, FLT *rotation, CnMat *jac) {
	FLT sign = q[0] < 0 ? -1 : 1;
	FLT w = q[0] * sign, v[3];
	scale3d(v, q + 1, sign);
	FLT s2 = dot3d(v, v), n2 = s2 + w * w, s = sqrt(s2);
	FLT angle = 2 * atan2(s, w);
	FLT factor = s > 1e-6 ? angle / s : 2 / w;
	FLT radial = s > 1e-6 ? (2 * w * s / n2 - angle) / (s2 * s) : -4 / (3 * w * w * w);
	if (rotation)
		scale3d(rotation, v, factor);
	if (jac) {
		for (int i = 0; i < 3; i++) {
			cnMatrixSet(jac, i, 0, sign * -2 * v[i] / n2);
			for (int j = 0; j < 3; j++)
				cnMatrixSet(jac, i, j + 1, sign * ((i == j ? factor : 0) + radial * v[i] * v[j]));
		}
	}
}

static void local_rotation_measurement(const SurvivePose *observed, const SurvivePose *predicted,
									 FLT *rotation, CnMat *state_jac, CnMat *observation_jac) {
	LinmathQuat inverse, relative;
	quatgetconjugate(inverse, predicted->Rot);
	quatrotateabout(relative, observed->Rot, inverse);
	CN_CREATE_STACK_MAT(log_jac, 3, 4);
	rotation_log_jacobian(relative, rotation, state_jac || observation_jac ? &log_jac : NULL);
	if (!state_jac && !observation_jac)
		return;
	for (int col = 0; col < 4; col++) {
		LinmathQuat basis = {0}, derivative;
		basis[col] = 1;
		if (state_jac) {
			quatgetconjugate(basis, basis);
			quatrotateabout(derivative, observed->Rot, basis);
			for (int row = 0; row < 3; row++) {
				FLT value = 0;
				for (int i = 0; i < 4; i++) value -= cnMatrixGet(&log_jac, row, i) * derivative[i];
				cnMatrixSet(state_jac, row, col, value);
			}
			basis[col] = 1;
		}
		if (observation_jac) {
			quatrotateabout(derivative, basis, inverse);
			for (int row = 0; row < 3; row++) {
				FLT value = 0;
				for (int i = 0; i < 4; i++) value += cnMatrixGet(&log_jac, row, i) * derivative[i];
				cnMatrixSet(observation_jac, row, col, value);
			}
		}
	}
}

static bool map_obs_data_local_rotation(void *user, const CnMat *Z, const CnMat *x, CnMat *y, CnMat *H) {
	const LocalRotationObservation *observation = user;
	const SurvivePose *predicted = (const SurvivePose *)cn_as_const_vector(x);
	FLT rotation[3];
	CN_CREATE_STACK_MAT(rotation_jac, 3, 4);
	local_rotation_measurement(observation->pose, predicted, rotation, H ? &rotation_jac : NULL, NULL);
	if (y) {
		sub3d(cn_as_vector(y), observation->pose->Pos, predicted->Pos);
		copy3d(cn_as_vector(y) + 3, rotation);
	}
	if (H) {
		SurviveKalmanTracker *tracker = observation->tracker;
		bool error_state = tracker->use_error_state && tracker->obs_model.error_state_model;
		CN_CREATE_STACK_MAT(H_full, 6, tracker->model.state_cnt);
		CnMat *H_pose = error_state ? &H_full : H;
		cn_set_zero(H_pose);
		for (int i = 0; i < 3; i++) {
			cnMatrixSet(H_pose, i, i, 1);
			for (int j = 0; j < 4; j++)
				cnMatrixSet(H_pose, i + 3, j + 3, cnMatrixGet(&rotation_jac, i, j));
		}
		if (error_state) {
			CN_CREATE_STACK_MAT(dx, tracker->model.state_cnt, tracker->model.error_state_size);
			state_update_fn(tracker, x, NULL, NULL, &dx);
			cnGEMM(&H_full, &dx, 1, NULL, 0, H, 0);
		}
	}
	return true;
}

static void observe_optical_motion(SurviveKalmanTracker *tracker, FLT time, const SurvivePose *pose, const CnMat *R) {
	FLT window = tracker->optical_motion_window;
	const unsigned capacity = sizeof(tracker->optical_motion_samples) / sizeof(tracker->optical_motion_samples[0]);
	if (window <= 0 || !isfinite(window)) return;
	if (tracker->optical_motion_start_time == 0) tracker->optical_motion_start_time = time;
	if (tracker->stats.obs_count < 16 ||
		time - tracker->last_unreliable_pose_time < tracker->zvu_motion_guard_time)
		return;
	if (tracker->optical_motion_count) {
		unsigned prior = (tracker->optical_motion_next + capacity - 1) % capacity;
		// Do not count overlapping optical solves as independent high-rate
		// velocity observations. This only controls stationary evidence.
		if (time - tracker->optical_motion_samples[prior].time < .0198) return;
	}
	unsigned next = tracker->optical_motion_next;
	tracker->optical_motion_samples[next].time = time;
	tracker->optical_motion_samples[next].pose = *pose;
	CnMat stored_covariance = cnMat(7, 7, tracker->optical_motion_samples[next].covariance);
	cnCopy(R, &stored_covariance, NULL);
	tracker->optical_motion_next = (next + 1) % capacity;
	if (tracker->optical_motion_count < capacity) tracker->optical_motion_count++;
	// Fit position and local rotation against time, using each observation's
	// full 3D marginal covariance. A velocity significantly different from
	// zero disproves the stationary hypothesis for that component. This fit
	// supplies detector evidence; it never replaces or averages output poses.
	for (int angular = 0; angular < 2; angular++) {
		CN_CREATE_STACK_MAT(normal, 6, 6);
		CN_CREATE_STACK_VEC(information, 6);
		unsigned count = 0;
		FLT span = 0;
		for (unsigned n = 0; n < tracker->optical_motion_count; n++) {
			FLT dt = tracker->optical_motion_samples[n].time - time;
			if (dt > 0 || -dt > window) continue;
			CnMat full_covariance = cnMat(7, 7, tracker->optical_motion_samples[n].covariance);
			CnMat covariance = cnMatView(3, 3, &full_covariance, 0, 0);
			CN_CREATE_STACK_MAT(rotation_covariance, 3, 3);
			FLT delta[3];
			if (angular) {
				CN_CREATE_STACK_MAT(rotation_jacobian, 3, 4);
				local_rotation_measurement(&tracker->optical_motion_samples[n].pose, pose,
					delta, NULL, &rotation_jacobian);
				CnMat quaternion_covariance = cnMatView(4, 4, &full_covariance, 3, 3);
				gemm_ABAt_add_scaled(&rotation_covariance, &rotation_jacobian, &quaternion_covariance, 0, 1, 1, 0);
				covariance = rotation_covariance;
			} else {
				sub3d(delta, tracker->optical_motion_samples[n].pose.Pos, pose->Pos);
			}
			CN_CREATE_STACK_MAT(weight, 3, 3);
			cnInvert(&covariance, &weight, CN_INVERT_METHOD_SVD);
			if (!cn_is_finite(&weight)) return;
			FLT factors[] = {1, dt};
			for (int block = 0; block < 2; block++) {
				for (int i = 0; i < 3; i++) {
					FLT value = 0;
					for (int j = 0; j < 3; j++) {
						FLT w = cnMatrixGet(&weight, i, j);
						value += w * delta[j];
						for (int other = 0; other < 2; other++) {
							int row = 3 * block + i, col = 3 * other + j;
							cnMatrixSet(&normal, row, col, cnMatrixGet(&normal, row, col) +
								factors[block] * factors[other] * w);
						}
					}
					int row = 3 * block + i;
					cnMatrixSet(&information, row, 0, cnMatrixGet(&information, row, 0) + factors[block] * value);
				}
			}
			span = linmath_max(span, -dt);
			count++;
		}
		if (count < 8 || span < window * .5) return;
		CN_CREATE_STACK_MAT(inverse, 6, 6);
		CN_CREATE_STACK_VEC(estimate, 6);
		cnInvert(&normal, &inverse, CN_INVERT_METHOD_SVD);
		cnGEMM(&inverse, &information, 1, NULL, 0, &estimate, 0);
		CnMat velocity_covariance = cnMatView(3, 3, &inverse, 3, 3);
		CN_CREATE_STACK_MAT(velocity_information, 3, 3);
		cnInvert(&velocity_covariance, &velocity_information, CN_INVERT_METHOD_SVD);
		FLT score = 0;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				score += cnMatrixGet(&estimate, i + 3, 0) * cnMatrixGet(&velocity_information, i, j) *
					cnMatrixGet(&estimate, j + 3, 0);
		if (isfinite(score) && score > 16) {
			tracker->optical_motion_until[angular] = time + window;
		}
	}
	tracker->optical_motion_fit_time = time;
}

static FLT integrate_pose(SurviveKalmanTracker *tracker, FLT time, const SurvivePose *pose, const struct CnMat *R_q) {
	FLT rtn = 0;

	size_t state_cnt = tracker->model.state_cnt;
	size_t obs_cnt = tracker->obs_local_rotation || tracker->obs_axisangle_model ? 6 : 7;

	struct CnMat* Rp = (CnMat*)R_q;
	CN_CREATE_STACK_MAT(R, obs_cnt, obs_cnt);
	CnMat adaptive_R = cnMat(obs_cnt, obs_cnt, tracker->Obs_R);
	CN_CREATE_STACK_VEC(Z, obs_cnt);
	LocalRotationObservation observation = {.tracker = tracker, .pose = pose};
	SurvivePose predicted = tracker->state.Pose;
	if (tracker->obs_local_rotation || !tracker->obs_axisangle_model) {
		survive_kalman_tracker_predict(tracker, time, &predicted);
		// Retain the stationary-constraint reliability check with either model.
		FLT qnorm = quatmagnitude(predicted.Rot) * quatmagnitude(pose->Rot);
		if (tracker->stats.obs_count < 16 || dist3d(predicted.Pos, pose->Pos) > .02 ||
			fabs(quatinnerproduct(predicted.Rot, pose->Rot)) < qnorm * cos(.05))
			tracker->last_unreliable_pose_time = time;
	}
	if (tracker->obs_local_rotation) {
		CN_CREATE_STACK_MAT(G, 6, 7);
		CN_CREATE_STACK_MAT(rotation_jac, 3, 4);
		local_rotation_measurement(pose, &predicted, NULL, NULL, &rotation_jac);
		for (int i = 0; i < 3; i++) {
			cnMatrixSet(&G, i, i, 1);
			for (int j = 0; j < 4; j++)
				cnMatrixSet(&G, i + 3, j + 3, cnMatrixGet(&rotation_jac, i, j));
		}
		if (R_q) {
			gemm_ABAt_add_scaled(&R, &G, R_q, 0, 1, 1, 0);
			Rp = &R;
		} else {
			Rp = &adaptive_R;
		}
		copy3d(cn_as_vector(&Z), pose->Pos);
	} else if(tracker->obs_axisangle_model) {
		LinmathAxisAnglePose poseAA = Pose2AAPose(pose);
		if(Rp) {
			survive_covariance_pose2poseAA(&R, pose, R_q);
		}
		memcpy(cn_as_vector(&Z), poseAA.Pos, obs_cnt * sizeof(FLT));
		Rp = &R;
	} else {
		memcpy(cn_as_vector(&Z), pose->Pos, obs_cnt * sizeof(FLT));
		if (tracker->obs_align_quaternion && quatinnerproduct(predicted.Rot, pose->Rot) < 0) {
			// q and -q encode the same orientation. Choose the measurement
			// hemisphere nearest the prior before forming a Euclidean residual.
			for (int i = 3; i < 7; i++)
				cnMatrixSet(&Z, i, 0, -cnMatrixGet(&Z, i, 0));
			if (Rp) {
				// Covariance must follow the same coordinate change: S R S^T,
				// S = diag(1,1,1,-1,-1,-1,-1). Rotation/position cross terms flip.
				cnCopy(R_q, &R, 0);
				for (int i = 0; i < 7; i++)
					for (int j = 0; j < 7; j++)
						if ((i < 3) != (j < 3))
							cnMatrixSet(&R, i, j, -cnMatrixGet(&R, i, j));
				Rp = &R;
			}
		}

	}

	tracker->datalog_tag = "pose_obs";
    rtn = cnkalman_meas_model_predict_update(time, &tracker->obs_model,
		tracker->obs_local_rotation ? (void *)&observation : (void *)tracker, &Z, Rp);

	tracker->datalog_tag = 0;
	SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(600, "Resultant state %f (pose) " Point16_format, time,
			   LINMATH_VEC16_EXPAND(cn_as_const_vector(&tracker->model.state)));

	return rtn;
}

enum {
	gyro_control_error_count = sizeof(SurviveKalmanErrorModel) / sizeof(FLT),
	gyro_control_omega = offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot) / sizeof(FLT),
	gyro_control_rotation = offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot) / sizeof(FLT),
	gyro_control_bias = offsetof(SurviveKalmanErrorModel, IMUBias.GyroBias) / sizeof(FLT),
	gyro_control_correction = offsetof(SurviveKalmanErrorModel, IMUBias.IMUCorrection) / sizeof(FLT)
};

static void gyro_control_transform_covariance(cnkalman_state_t *model,
											const double *G) {
	double prior[gyro_control_error_count * gyro_control_error_count];
	double product[gyro_control_error_count * gyro_control_error_count] = {0};
	double posterior[gyro_control_error_count * gyro_control_error_count] = {0};
	const int n = gyro_control_error_count;
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++) prior[n * i + j] = cnMatrixGet(&model->P, i, j);
	for (int i = 0; i < n; i++)
		for (int k = 0; k < n; k++)
			if (G[n * i + k] != 0)
				for (int j = 0; j < n; j++) product[n * i + j] += G[n * i + k] * prior[n * k + j];
	for (int j = 0; j < n; j++)
		for (int k = 0; k < n; k++)
			if (G[n * j + k] != 0)
				for (int i = 0; i < n; i++) posterior[n * i + j] += product[n * i + k] * G[n * j + k];
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
			cnMatrixSet(&model->P, i, j, .5 * (posterior[n * i + j] + posterior[n * j + i]));
}

static bool integrate_gyro_control(SurviveKalmanTracker *tracker, FLT time, const FLT gyro[3]) {
	if (!tracker->gyro_control_mode || !tracker->use_gyro_control || tracker->gyro_var < 0 ||
		tracker->imu_model.adaptive || !tracker->imu_model.error_state_model || tracker->use_raw_obs ||
		tracker->noise_model != 0 ||
		fabs(fabs(tracker->state.IMUBias.IMUCorrection[0]) - 1) > 1e-12 ||
		cnMatrixGet(&tracker->model.P, gyro_control_correction, gyro_control_correction) != 0 ||
		cnMatrixGet(&tracker->model.P, gyro_control_correction + 1, gyro_control_correction + 1) != 0 ||
		cnMatrixGet(&tracker->model.P, gyro_control_correction + 2, gyro_control_correction + 2) != 0) {
		tracker->gyro_control_started = false;
		return false;
	}
	if (time < tracker->model.t || time <= tracker->gyro_control_time) return false;

	// The previous gyro sample, with its posterior uncertainty, controls the
	// complete interval up to this acquisition. Intermediate optical updates
	// must not invent independent angular process noise or change that interval.
	tracker->gyro_control_started = true;
	cnkalman_predict_state(time, &tracker->model);

	double w = tracker->state.Pose.Rot[0], x = tracker->state.Pose.Rot[1];
	double y = tracker->state.Pose.Rot[2], z = tracker->state.Pose.Rot[3];
	double norm = w * w + x * x + y * y + z * z;
	double R[9] = {
		1 - 2 * (y * y + z * z) / norm, 2 * (x * y - z * w) / norm, 2 * (x * z + y * w) / norm,
		2 * (x * y + z * w) / norm, 1 - 2 * (x * x + z * z) / norm, 2 * (y * z - x * w) / norm,
		2 * (x * z - y * w) / norm, 2 * (y * z + x * w) / norm, 1 - 2 * (x * x + y * y) / norm
	};
	double corrected[3];
	for (int i = 0; i < 3; i++) corrected[i] = gyro[i] - tracker->state.IMUBias.GyroBias[i];
	double skew[9] = {0, -corrected[2], corrected[1], corrected[2], 0, -corrected[0],
					 -corrected[1], corrected[0], 0};
	double G[gyro_control_error_count * gyro_control_error_count] = {0};
	const int n = gyro_control_error_count, omega = gyro_control_omega;
	for (int i = 0; i < n; i++) G[n * i + i] = 1;
	for (int i = 0; i < 3; i++) {
		memset(G + n * (omega + i), 0, n * sizeof(*G));
		for (int j = 0; j < 3; j++) {
			double value = 0;
			for (int k = 0; k < 3; k++) value -= R[3 * i + k] * skew[3 * k + j];
			G[n * (omega + i) + gyro_control_rotation + j] = value;
			G[n * (omega + i) + gyro_control_bias + j] = -R[3 * i + j];
		}
	}
	// omega_world = R(q) * (gyro_measured - bias). Keep its rotation/bias
	// correlations, replace the previous sample's noise with this independent
	// sample, and let subsequent predictions transport the whole covariance.
	gyro_control_transform_covariance(&tracker->model, G);
	for (int i = 0; i < 3; i++) {
		double value = 0;
		for (int j = 0; j < 3; j++) value += R[3 * i + j] * corrected[j];
		tracker->state.Velocity.AxisAngleRot[i] = value;
		for (int j = 0; j < 3; j++) {
			double variance = 0;
			for (int k = 0; k < 3; k++) variance += tracker->gyro_var * R[3 * i + k] * R[3 * j + k];
			cnMatrixSet(&tracker->model.P, omega + i, omega + j,
				cnMatrixGet(&tracker->model.P, omega + i, omega + j) + variance);
		}
	}
	tracker->gyro_control_time = time;
	return true;
}

void survive_kalman_tracker_integrate_imu(SurviveKalmanTracker *tracker, PoserDataIMU *data) {
	SurviveContext *ctx = tracker->so->ctx;
	SurviveObject *so = tracker->so;

	FLT time = data->hdr.timecode / (FLT)tracker->so->timebase_hz;
	FLT time_diff = time - tracker->model.t;

	FLT norm = norm3d(data->accel);
	SV_DATA_LOG("acc_norm", &norm, 1);

	bool isStationary = SurviveSensorActivations_stationary_time(&tracker->so->activations) > 4800000;

	if (tracker->use_raw_obs) {
		return;
	}

	// Wait til observation is in before reading IMU; gets rid of bad IMU data at the start
	if (tracker->model.t == 0) {
		return;
	}

	if (tracker->stats.obs_count < 16 && tracker->obs_pos_var > -1) {
		return;
	}

	if (time_diff < -.01) {
		// SV_WARN("Processing imu data from the past %fs", time - tracker->rot.t);
		tracker->stats.late_imu_dropped++;
		return;
	}

	if (time_diff > 0.5) {
		SV_WARN("%s is probably dropping IMU packets; %f time reported between %" PRIu64, tracker->so->codename,
				time_diff, data->hdr.timecode);
	}

	bool gyro_control = integrate_gyro_control(tracker, time, data->gyro);

	FLT rotation_variance[] = {1e5, 1e5, 1e5, 1e5, 1e5, 1e5};

	bool no_light = (time - tracker->last_light_time) > tracker->zvu_no_light_time;
	FLT zvu_var = tracker->zvu_moving_var;
	FLT zvu_velocity_var = zvu_var;
	FLT zvu_acceleration_var = zvu_var;
	if (isStationary && tracker->zvu_stationary_var >= 0) {
		FLT stationary_var = tracker->zvu_stationary_var;
		FLT remaining_motion = 0;
		bool motion_target = so->object_type == SURVIVE_OBJECT_TYPE_OTHER || tracker->zvu_motion_all_devices;
		bool settled = motion_target && time - tracker->last_unreliable_pose_time >= tracker->zvu_motion_guard_time;
		if ((settled || (motion_target && !no_light)) && tracker->zvu_stationary_ramp > 0) {
			// A short pause in handheld motion is weak evidence of a zero
			// velocity. Introduce measurement information continuously rather
			// than applying the full constraint at the first 100 ms crossing.
			// Uncertain optical poses do not strengthen that rest evidence.
			// Retain the previous behavior when light is also unavailable.
			// Keep the ordinary Kalman gain/covariance update together.
			FLT stationary_s = SurviveSensorActivations_stationary_time(&so->activations) / (FLT)so->timebase_hz;
			FLT u = linmath_enforce_range((stationary_s - .1) / tracker->zvu_stationary_ramp, 0, 1);
			FLT information = u * u * (3 - 2 * u);
			stationary_var = information > 0 ? stationary_var / information : INFINITY;
			remaining_motion = 1 - u;
		}
		if (settled && tracker->zvu_stationary_gyro_scale > 0 && tracker->gyro_var >= 0) {
			// The activation detector tolerates slow rotation. A zero-velocity
			// observation must remain weak when the bias-corrected IMU actually
			// sees that motion, even if it has not reset the detector's timer.
			FLT rate2 = 0;
			for (int axis = 0; axis < 3; axis++) {
				FLT rate = (data->gyro[axis] - tracker->state.IMUBias.GyroBias[axis]) / tracker->zvu_stationary_gyro_scale;
				rate2 += rate * rate;
			}
			stationary_var *= 1 + rate2 * rate2;
		}
		if (isfinite(stationary_var)) {
			zvu_var = linmath_min(stationary_var, zvu_var < 0 ? INFINITY : zvu_var);
			// Quiet accelerations do not establish zero translation velocity.
			// Allow slow motion while rest evidence accumulates. Keeping this
			// uncertainty in R also keeps the state and covariance consistent.
			FLT velocity_sigma = tracker->zvu_stationary_velocity_sigma * remaining_motion;
			zvu_velocity_var = linmath_min(stationary_var + velocity_sigma * velocity_sigma,
			                               zvu_velocity_var < 0 ? INFINITY : zvu_velocity_var);
			FLT acceleration_sigma = tracker->zvu_stationary_ramp > 0 ? velocity_sigma / tracker->zvu_stationary_ramp : 0;
			zvu_acceleration_var = linmath_min(stationary_var + acceleration_sigma * acceleration_sigma,
			                                   zvu_acceleration_var < 0 ? INFINITY : zvu_acceleration_var);
		}
	}
	if (no_light && tracker->zvu_no_light_var >= 0) {
		zvu_var = linmath_min(tracker->zvu_no_light_var, zvu_var < 0 ? INFINITY : zvu_var);
		zvu_velocity_var = linmath_min(tracker->zvu_no_light_var, zvu_velocity_var < 0 ? INFINITY : zvu_velocity_var);
		zvu_acceleration_var = linmath_min(tracker->zvu_no_light_var, zvu_acceleration_var < 0 ? INFINITY : zvu_acceleration_var);
	}
	FLT zvu_angular_var = zvu_var;
	if (!no_light && tracker->optical_motion_window > 0) {
		// Give the detector time to initialize before assuming a zero angular
		// velocity. Bound that grace period: persistently unreliable optics
		// cannot leave the gyro's stationary bias constraint disabled forever.
		if (tracker->optical_motion_fit_time <= 0 && tracker->optical_motion_start_time > 0 &&
			time - tracker->optical_motion_start_time < 2 * tracker->optical_motion_window + tracker->zvu_motion_guard_time)
			zvu_angular_var = -1;
		for (int angular = 0; angular < 2; angular++) {
			if (tracker->optical_motion_until[angular] <= 0) continue;
			FLT quiet = time - tracker->optical_motion_until[angular];
			FLT u = tracker->zvu_stationary_ramp > 0 ?
				linmath_enforce_range((quiet - .1) / tracker->zvu_stationary_ramp, 0, 1) : (quiet > .1);
			FLT information = u * u * (3 - 2 * u);
			if (angular) {
				zvu_angular_var = information > 0 ? zvu_angular_var / information : -1;
			} else {
				zvu_velocity_var = information > 0 ? zvu_velocity_var / information : -1;
				zvu_acceleration_var = information > 0 ? zvu_acceleration_var / information : -1;
			}
		}
	}

	bool disable_ang_vel = no_light && !isStationary;
	tracker->stats.no_light_imu_count += no_light;

	int indices[9] = {0}, row_cnt = 0;
	FLT variances[9] = {0};
	int starts[] = {offsetof(SurviveKalmanModel, Velocity.Pos) / sizeof(FLT),
		offsetof(SurviveKalmanModel, Velocity.AxisAngleRot) / sizeof(FLT),
		offsetof(SurviveKalmanModel, Acc) / sizeof(FLT)};
	FLT group_variance[] = {zvu_velocity_var, disable_ang_vel ? -1 : zvu_angular_var, zvu_acceleration_var};
	for (int group = 0; group < 3; group++)
		if (group_variance[group] >= 0 && isfinite(group_variance[group]))
			for (int axis = 0; axis < 3; axis++)
				if (starts[group] + axis < tracker->model.state_cnt) {
					indices[row_cnt] = starts[group] + axis;
					variances[row_cnt++] = group_variance[group];
				}
	if (row_cnt) {
		CN_CREATE_STACK_MAT(H, row_cnt, tracker->model.state_cnt + tracker->imu_bias_model.state_cnt);
		for (int row = 0; row < row_cnt; row++) cnMatrixSet(&H, row, indices[row], 1);
		CN_CREATE_STACK_MAT(R, row_cnt, 1)
		for (int row = 0; row < row_cnt; row++) cnMatrixSet(&R, row, 0, variances[row]);
		CN_CREATE_STACK_MAT(Z, row_cnt, 1);
		cn_set_zero(&Z);

		tracker->datalog_tag = "zvu";

		tracker->stats.imu_total_error +=
			cnkalman_meas_model_predict_update(time, &tracker->zvu_model, &H, &Z, &R);

		tracker->datalog_tag = 0;

		CN_FREE_STACK_MAT(Z);
		CN_FREE_STACK_MAT(R);
		CN_FREE_STACK_MAT(H);
	}

	struct map_imu_data_ctx fn_ctx = {.tracker = tracker};
	if (tracker->acc_var >= 0) {
		fn_ctx.use_accel = true;
		for (int i = 0; i < 3; i++) {
			rotation_variance[i] = tracker->acc_var;
			if (tracker->acc_norm_penalty > 0) {
			  FLT ndiff = 1 - norm;
			  rotation_variance[i] += tracker->acc_norm_penalty * ndiff * ndiff;
			}
		}
	}

	if (tracker->gyro_var >= 0) {
		fn_ctx.use_gyro = true;
		for (int i = 0; i < 3; i++)
			rotation_variance[3 + i] = tracker->gyro_var;
	}

	if (fn_ctx.use_gyro || fn_ctx.use_accel) {
		int rows = gyro_control ? 3 : 6;
		int offset = 0;
		FLT accelgyro[6] = { 0 };
		copy3d(accelgyro, data->accel);
		copy3d(accelgyro+3, data->gyro);

		integrate_variance_tracker(tracker, &tracker->imu_variance, accelgyro, 6);

		CnMat Z = cnMat(rows, 1, accelgyro + offset);

		SV_VERBOSE(600, "Integrating IMU " Point6_format " with cov " Point6_format,
				   LINMATH_VEC6_EXPAND((FLT *)&accelgyro[0]), LINMATH_VEC6_EXPAND(rotation_variance));

		tracker->datalog_tag = "imu_meas";

		CnMat R = cnMat(rows, tracker->imu_model.adaptive ? rows : 1,
			tracker->imu_model.adaptive ? tracker->IMU_R : rotation_variance);
		FLT err = 0;
		if (!gyro_control || fn_ctx.use_accel) {
			kalman_measurement_model_fn_t original = tracker->imu_model.Hfn;
			if (gyro_control) tracker->imu_model.Hfn = accelerometer_measurement_model;
			err = cnkalman_meas_model_predict_update(time, &tracker->imu_model, &fn_ctx, &Z, &R);
			tracker->imu_model.Hfn = original;
		}
		tracker->datalog_tag = 0;

        SV_DATA_LOG("res_err_imu", &err, 1);
		tracker->stats.imu_total_error += err;
		tracker->imu_residuals *= .9;
		tracker->imu_residuals += .1 * err;

        tracker->stats.acc_norm += norm3d(data->accel);
        if(isStationary) {
            tracker->stats.stationary_acc_norm += norm3d(data->accel);
            tracker->stats.stationary_imu_count++;
        }
		tracker->stats.imu_count++;
		if (tracker->first_imu_time == 0) {
		  tracker->first_imu_time = time;
		}
		
		tracker->last_imu_time = time;

		SV_VERBOSE(600, "%s Resultant state %f (imu %e) " Point26_format, so->codename, time, tracker->imu_residuals,
				   LINMATH_VEC26_EXPAND(cn_as_const_vector(&tracker->model.state)));
	}

	survive_kalman_tracker_report_state(&data->hdr, tracker);
}

void survive_kalman_tracker_predict(const SurviveKalmanTracker *tracker, FLT t, SurvivePose *out) {
	// if (tracker->model.info.P[0] > 100 || tracker->model.info.P[0] > 100 || tracker->model.t == 0)
	//	return;

	if (tracker->model.t == 0)
		return;

	CnMat x1 = cnVec(7, out->Pos);
	cnkalman_extrapolate_state(t, &tracker->model, &x1, 0);
	quatnormalize(out->Rot, out->Rot);

	struct SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(300, "Predict pose %f %f " SurvivePose_format, t, t - tracker->model.t, SURVIVE_POSE_EXPAND(*out))
}

// Add random walk in the covariance that actually owns the IMU bias.
static void imu_bias_process_noise(void *user, FLT dt, const CnMat *state, CnMat *out) {
	const struct SurviveKalmanTracker_Params *params = user;
	cn_set_zero(out);
	int start = out->rows == sizeof(SurviveKalmanErrorModel) / sizeof(FLT) ?
		offsetof(SurviveKalmanErrorModel, IMUBias) / sizeof(FLT) : 0;
	for (int axis = 0; axis < 3; axis++) {
		int accel = start + offsetof(SurviveIMUBiasErrorModel, AccBias) / sizeof(FLT) + axis;
		int gyro = start + offsetof(SurviveIMUBiasErrorModel, GyroBias) / sizeof(FLT) + axis;
		if (accel < out->rows)
			cnMatrixSet(out, accel, accel, params->process_weight_acc_bias * dt);
		if (gyro < out->rows)
			cnMatrixSet(out, gyro, gyro, params->process_weight_gyro_bias * dt);
	}
}

static void survive_kalman_tracker_process_noise_bounce(void *user, FLT t, const CnMat *x, struct CnMat *q_out) {
	struct SurviveKalmanTracker_Params *params = (struct SurviveKalmanTracker_Params *)user;
	survive_kalman_tracker_process_noise(params, false, t, x, q_out);
}

static void survive_kalman_tracker_error_process_noise_bounce(void *user, FLT t, const CnMat *x, struct CnMat *q_out) {
	struct SurviveKalmanTracker_Params *params = (struct SurviveKalmanTracker_Params *)user;
	survive_kalman_tracker_process_noise(params, true, t, x, q_out);
}

static void gyro_control_process_noise(void *user, FLT t, const CnMat *x, CnMat *out) {
	SurviveKalmanTracker *tracker = user;
	struct SurviveKalmanTracker_Params params = tracker->params;
	if (tracker->gyro_control_started && tracker->use_gyro_control && tracker->gyro_var >= 0 &&
		!tracker->imu_model.adaptive && tracker->imu_model.error_state_model)
		params.process_weight_ang_velocity = 0;
	survive_kalman_tracker_process_noise(&params, true, t, x, out);
}

void survive_kalman_tracker_process_noise(const struct SurviveKalmanTracker_Params *params, bool errorState, FLT t, const CnMat *x, struct CnMat *q_out) {
	/*
	 * Due to the rotational terms in the model, the process noise covariance is complicated. It mixes a XYZ third order
	 * positional model with a second order rotational model with tuning parameters
	 */

	/* dt is capped to prevent NaN/inf values */
	if (t > 0.05) t = 0.05;

	FLT t2 = t * t;
	FLT t3 = t2 * t;
	FLT t4 = t3 * t;
	FLT t5 = t4 * t;
	FLT t6 = t5 * t;
	FLT t7 = t6 * t;
	/* ================== Positional ============================== */
	// Estimation with Applications to Tracking and Navigation: Theory Algorithms and Software Ch 6
	// http://wiki.dmdevelopment.ru/wiki/Download/Books/Digitalimageprocessing/%D0%9D%D0%BE%D0%B2%D0%B0%D1%8F%20%D0%BF%D0%BE%D0%B4%D0%B1%D0%BE%D1%80%D0%BA%D0%B0%20%D0%BA%D0%BD%D0%B8%D0%B3%20%D0%BF%D0%BE%20%D1%86%D0%B8%D1%84%D1%80%D0%BE%D0%B2%D0%BE%D0%B9%20%D0%BE%D0%B1%D1%80%D0%B0%D0%B1%D0%BE%D1%82%D0%BA%D0%B5%20%D1%81%D0%B8%D0%B3%D0%BD%D0%B0%D0%BB%D0%BE%D0%B2/Estimation%20with%20Applications%20to%20Tracking%20and%20Navigation/booktext@id89013302placeboie.pdf

	// Q_acc integrates continuous white noise on acceleration for the [p,v,a]
	// state. It composes across prediction intervals, including when an optical
	// update splits an IMU interval. Q_jerk is retained for old configurations;
	// its higher-order process would require a jerk state to have that property.
	FLT Q_jerk[] = {
		t7 / 252.,
		t6 / 72., t5 /20.,
		t5 / 30, t4 / 8., t3 / 3.
	};
	FLT Q_acc[] = {
		t5 / 20.,
		t4 / 8.,      t3 / 3.,
		t3 / 6.,      t2 / 2.,       t
	};
	FLT Q_vel[] = {
		t3 / 3.,
		t2 / 2.,       t,
	};

	FLT p_p = params->process_weight_jerk * Q_jerk[0] + params->process_weight_acc * Q_acc[0] + params->process_weight_vel * Q_vel[0] + params->process_weight_pos * t2;
	FLT p_v = params->process_weight_jerk * Q_jerk[1] + params->process_weight_acc * Q_acc[1] + params->process_weight_vel * Q_vel[1];
	FLT p_a = params->process_weight_jerk * Q_jerk[3] + params->process_weight_acc * Q_acc[3];

	FLT v_v = params->process_weight_jerk * Q_jerk[2] + params->process_weight_acc * Q_acc[2] + params->process_weight_vel * Q_vel[2];
	FLT v_a = params->process_weight_jerk * Q_jerk[4] + params->process_weight_acc * Q_acc[4];
	FLT a_a = params->process_weight_jerk * Q_jerk[5] + params->process_weight_acc * Q_acc[5];


	/* ================== Rotational ==============================
	 * 	https://www.ucalgary.ca/engo_webdocs/GL/96.20096.JSchleppe.pdf
	 *      !!! NOTE: This document uses x,y,z,w quaternions !!!
	  This is a rework using the same methodology. Some helper output functions are in the tools/generate_math_functions
	  code.
	 */
	FLT s_w = params->process_weight_ang_velocity;
	FLT rv =   params->process_weight_ang_velocity * Q_vel[0] + params->process_weight_rotation * t;
	FLT r_av = params->process_weight_ang_velocity * Q_vel[1];

	/* The gyro bias is expected to change, but slowly through time */
	FLT ga = params->process_weight_acc_bias * t;
	FLT gb = params->process_weight_gyro_bias * t;

	if(!errorState) {
		FLT s_f = s_w / 12. * t3;
		FLT s_s = s_w / 4. * t2;

		size_t state_cnt = x->rows;
		SurviveKalmanModel state = copy_model(cn_as_const_vector(x), state_cnt);

		FLT qw = state.Pose.Rot[0], qx = state.Pose.Rot[1], qy = state.Pose.Rot[2], qz = state.Pose.Rot[3];
		FLT qws = qw * qw, qxs = qx * qx, qys = qy * qy, qzs = qz * qz;
		FLT qs = qws + qxs + qys + qzs;

		FLT Q_POSE_BLOCK[] = {
	//       x        y        z                 qw                 qx                 qy                 qz         vx       vy       vz          avx      avy      avz       ax       ay      az
		  p_p,       0,       0,                 0,                 0,                 0,                 0,       p_v,       0,       0,           0,       0,       0,     p_a,       0,       0,  // x
			0,     p_p,       0,                 0,                 0,                 0,                 0,         0,     p_v,       0,           0,       0,       0,       0,     p_a,       0,  // y
			0,       0,     p_p,                 0,                 0,                 0,                 0,         0,       0,     p_v,           0,       0,       0,       0,       0,     p_a,  // z

			0,       0,       0,   rv+s_f*(qs-qws),      s_f*(-qw*qx),      s_f*(-qw*qy),      s_f*(-qw*qz),         0,       0,       0,     -s_s*qx, -s_s*qy, -s_s*qz,       0,       0,       0,  // qw
			0,       0,       0,      s_f*(-qw*qx),   rv+s_f*(qs-qxs),      s_f*(-qx*qy),      s_f*(-qx*qz),         0,       0,       0,      s_s*qw, -s_s*qz,  s_s*qy,       0,       0,       0,  // qx
			0,       0,       0,      s_f*(-qw*qy),      s_f*(-qx*qy),   rv+s_f*(qs-qys),      s_f*(-qy*qz),         0,       0,       0,      s_s*qz,  s_s*qw, -s_s*qx,       0,       0,       0,  // qy
			0,       0,       0,      s_f*(-qw*qz),      s_f*(-qx*qz),      s_f*(-qy*qz),   rv+s_f*(qs-qzs),         0,       0,       0,     -s_s*qy,  s_s*qx,  s_s*qw,       0,       0,       0,  // qz

		  p_v,       0,       0,                 0,                 0,                 0,                 0,       v_v,       0,       0,           0,       0,       0,     v_a,       0,       0,  // vx
			0,     p_v,       0,                 0,                 0,                 0,                 0,         0,     v_v,       0,           0,       0,       0,       0,     v_a,       0,  // vy
			0,       0,     p_v,                 0,                 0,                 0,                 0,         0,       0,     v_v,           0,       0,       0,       0,       0,     v_a,  // vz

			0,       0,       0,           -s_s*qx,            s_s*qw,            s_s*qz,           -s_s*qy,         0,       0,       0,     s_w * t,       0,       0,       0,       0,       0,  // avx
			0,       0,       0,           -s_s*qy,           -s_s*qz,            s_s*qw,            s_s*qx,         0,       0,       0,           0, s_w * t,       0,       0,       0,       0,  // avy
			0,       0,       0,           -s_s*qz,            s_s*qy,           -s_s*qx,            s_s*qw,         0,       0,       0,           0,       0, s_w * t,       0,       0,       0,  // avz

		  p_a,       0,       0,                 0,                 0,                 0,                 0,       v_a,       0,       0,           0,       0,       0,     a_a,       0,       0,  // ax
			0,     p_a,       0,                 0,                 0,                 0,                 0,         0,     v_a,       0,           0,       0,       0,       0,     a_a,       0,  // ay
			0,       0,     p_a,                 0,                 0,                 0,                 0,         0,       0,     v_a,           0,       0,       0,       0,       0,     a_a,  // az

		};
		cn_copy_in_row_major_roi(q_out, Q_POSE_BLOCK, 16, 0, 0, 16, 16);
	} else {

		FLT Q_POSE_BLOCK[] = {
     //       x        y        z           qx         qy       qz              vx       vy       vz          avx      avy      avz       ax       ay      az
			p_p,       0,       0,           0,         0,      0,            p_v,       0,       0,           0,       0,       0,     p_a,       0,       0,  // x
			0,       p_p,       0,           0,         0,      0,              0,     p_v,       0,           0,       0,       0,       0,     p_a,       0,  // y
			0,         0,     p_p,           0,         0,      0,              0,       0,     p_v,           0,       0,       0,       0,       0,     p_a,  // z

			0,         0,       0,          rv,         0,      0,              0,       0,       0,           r_av,    0,       0,       0,       0,       0,  // qx
			0,         0,       0,           0,        rv,      0,              0,       0,       0,           0,       r_av,    0,       0,       0,       0,  // qy
			0,         0,       0,           0,         0,     rv,              0,       0,       0,           0,       0,    r_av,       0,       0,       0,  // qz

			p_v,       0,       0,           0,         0,      0,            v_v,       0,       0,           0,       0,       0,     v_a,       0,       0,  // vx
			0,       p_v,       0,           0,         0,      0,              0,     v_v,       0,           0,       0,       0,       0,     v_a,       0,  // vy
			0,         0,     p_v,           0,         0,      0,              0,       0,     v_v,           0,       0,       0,       0,       0,     v_a,  // vz

			0,         0,       0,           r_av,      0,      0,              0,       0,       0,     s_w * t,        0,       0,       0,       0,       0,  // avx
			0,         0,       0,           0,      r_av,      0,              0,       0,       0,           0,  s_w * t,       0,       0,       0,       0,  // avy
			0,         0,       0,           0,         0,   r_av,              0,       0,       0,           0,        0, s_w * t,       0,       0,       0,  // avz

			p_a,       0,       0,           0,         0,      0,            v_a,       0,       0,           0,        0,       0,     a_a,       0,       0,  // ax
			0,       p_a,       0,           0,         0,      0,              0,     v_a,       0,           0,        0,       0,       0,     a_a,       0,  // ay
			0,         0,     p_a,           0,         0,      0,              0,       0,     v_a,           0,        0,       0,       0,       0,     a_a,  // az

		};
		cn_copy_in_row_major_roi(q_out, Q_POSE_BLOCK, 15, 0, 0, 15, 15);
		// Quaternion errors are local right perturbations, while angular
		// velocity uses world axes. Their integrated-noise correlation needs
		// R(q)^T; an identity cross block makes it depend on the world frame.
		SurviveKalmanModel state = copy_model(cn_as_const_vector(x), x->rows);
		LinmathQuat world_to_local;
		quatgetconjugate(world_to_local, state.Pose.Rot);
		for (int axis = 0; axis < 3; axis++) {
			LinmathVec3d basis = {0}, local;
			basis[axis] = 1;
			quatrotatevector(local, world_to_local, basis);
			for (int row = 0; row < 3; row++)
				if (row + 3 < q_out->rows && axis + 9 < q_out->cols) {
					cnMatrixSet(q_out, row + 3, axis + 9, r_av * local[row]);
					cnMatrixSet(q_out, axis + 9, row + 3, r_av * local[row]);
				}
		}
	}

	assert(cn_is_symmetrical(q_out));

	for(int i = 0;i < 3;i++) {
        int accBiasIdx = (int)(errorState ? offsetof(SurviveKalmanErrorModel , IMUBias.AccBias) : offsetof(SurviveKalmanModel, IMUBias.AccBias))/sizeof(FLT) + i;
        if(accBiasIdx < q_out->rows) cnMatrixSet(q_out, accBiasIdx, accBiasIdx, ga);

        int gyroBiasIdx = (int)(errorState ? offsetof(SurviveKalmanErrorModel, IMUBias.GyroBias) : offsetof(SurviveKalmanModel, IMUBias.GyroBias))/sizeof(FLT) + i;
		if(gyroBiasIdx < q_out->rows) cnMatrixSet(q_out, gyroBiasIdx, gyroBiasIdx, gb);
	}

}

/**
 * The prediction model and associated F matrix use generated code to simplifiy the jacobian. This might not be strictly
 * necessary but allows for quicker development.
 */
void survive_kalman_tracker_predict_jac(FLT dt, const struct cnkalman_state_s *k, const struct CnMat *x0, struct CnMat *x1, struct CnMat *f) {
	SurviveKalmanModel s_in = copy_model(cn_as_const_vector(x0), x0->rows);

	if(x1) {
		SurviveKalmanModel s_out = {0};

		// Noise densities describe uncertainty, not the estimated motion.
		// A zero density still predicts using the retained velocity/acceleration,
		// just as the error-state model does. Omitted state coordinates are
		// already zero-filled by copy_model.
		quatnormalize(s_in.Pose.Rot, s_in.Pose.Rot);
		SurviveKalmanModelPredict(&s_out, dt, &s_in);
		quatnormalize(s_out.Pose.Rot, s_out.Pose.Rot);

		memcpy(cn_as_vector(x1), s_out.Pose.Pos, x1->rows * sizeof(FLT));
	}

	if(f) {
		size_t state_cnt = x0->rows;
		if (dt == 0) {
			cn_eye(f, 0);
		} else {
			SurviveKalmanModelPredict_jac_kalman_model(f, dt, &s_in);
		}
	}
}

void survive_show_covariance(SurviveObject *so, const SurvivePose *pose, const struct CnMat *Ri, FLT s, FLT stddev) {
    SurviveContext *ctx = so->ctx;
    CN_CREATE_STACK_MAT(R, 7, 7);
    cnCopy(Ri, &R, 0);
    CN_CREATE_STACK_MAT(RL, 7, 7);
    CN_CREATE_STACK_MAT(X, 7, 1);
    CN_CREATE_STACK_MAT(Xs, 7, 1);
    cnSqRootSymmetric(&R, &RL);
    for(int i = 0;i < 25;i++) {
        cnRand(&X, 0, stddev);
        cnGEMM(&RL, &X, 1, 0, 0, &Xs, 0);
        addnd(_Xs, _Xs, pose->Pos, 7);
		quatnormalize(&_Xs[3], &_Xs[3]);
        char external_name[16] = {0};
        sprintf(external_name, "%s-sample_%d", so->codename, i);
        SurvivePose head2world = *pose;
        ApplyPoseToPose(&head2world, (SurvivePose *)_Xs, &so->head2imu);

        survive_recording_write_to_output(ctx->recptr, "AXIS %s_sample_%i_%f %f " Point7_format "\n",
                                          so->codename, i, s, s, SURVIVE_POSE_EXPAND(head2world));
    }
}

void survive_kalman_tracker_integrate_observation(PoserData *pd, SurviveKalmanTracker *tracker, const SurvivePose *pose,
												  const struct CnMat *Ri) {
	SurviveObject *so = tracker->so;
    SurviveContext *ctx = so->ctx;
	if (tracker->obs_local_rotation) {
		// The rotation logarithm has no value for a zero quaternion. Reject
		// malformed solver output before updating clocks, covariance or state.
		for (int i = 0; i < 3; i++)
			if (!isfinite(pose->Pos[i])) return;
		for (int i = 0; i < 4; i++)
			if (!isfinite(pose->Rot[i])) return;
		if (quatmagnitude(pose->Rot) < 1e-12) return;
	}

	integrate_variance_tracker(tracker, &tracker->pose_variance, (FLT*)pose->Pos, 7);

    if (tracker->show_raw_obs) {
        static int report_in_imu = -1;
        if (report_in_imu == -1) {
            report_in_imu = survive_configi(so->ctx, "report-in-imu", SC_GET, 0);
        }

        char external_name[16] = {0};
		sprintf(external_name, "%s-raw-obs", so->codename);
		SurvivePose head2world = *pose;
		if(!report_in_imu) {
            ApplyPoseToPose(&head2world, pose, &so->head2imu);
		}
        // The external-pose callback has no sensor timestamp. Keep the exact
        // observation epoch beside this opt-in diagnostic, before any fusion.
        survive_recording_write_to_output(ctx->recptr, "%s OPTICAL_TIMING %" PRIu64 " %d\n",
                                          external_name, pd->timecode, so->timebase_hz);
        SURVIVE_INVOKE_HOOK(external_pose, ctx, external_name, &head2world);
	}

	if (tracker->use_raw_obs) {
		SURVIVE_INVOKE_HOOK_SO(imupose, so, pd->timecode, pose);
		return;
	}

	survive_long_timecode timecode = pd->timecode;

	FLT time = timecode / (FLT)tracker->so->timebase_hz;
	if (tracker->model.t == 0) {
		tracker->model.t = time;
	}

	/*
	FLT R[] = {tracker->obs_pos_var, tracker->obs_pos_var, tracker->obs_pos_var, tracker->obs_rot_var,
			   tracker->obs_rot_var, tracker->obs_rot_var, tracker->obs_rot_var};
	if (oR) {
		addnd(R, R, oR, 7);
	}
*/
	if (time - tracker->model.t < 0) {
		if (time - tracker->model.t > -.1) {
			FLT tdiff = tracker->model.t - time;

			// Scale up the covariance
			FLT pS = 10, rS = 1;
			FLT Raug[] = {pS * tdiff, pS * tdiff, pS * tdiff, rS * tdiff, rS * tdiff, rS * tdiff, rS * tdiff};
			// addnd(R, R, Raug, 7);

			time = tracker->model.t;
		} else {
			// CN_WARN("Processing light data from the past %fs", time - tracker->model.t );
			tracker->stats.late_light_dropped++;
			return;
		}
	}

	tracker->last_light_time = time;

	if (tracker->obs_pos_var >= 0 && tracker->obs_rot_var >= 0) {
        CN_CREATE_STACK_MAT(R, 7, 7);
        if(Ri) {
            cnScale(&R, Ri, tracker->obs_cov_scale);
        }
        FLT augR[] = {tracker->obs_pos_var, tracker->obs_pos_var, tracker->obs_pos_var, tracker->obs_rot_var,
                      tracker->obs_rot_var, tracker->obs_rot_var, tracker->obs_rot_var};
        for(int i =0;i < 7;i++)
            cnMatrixSet(&R, i, i, cnMatrixGet(&R, i, i) + augR[i]);

        if(tracker->report_covariance_cnt > 0 && Ri && Ri->rows == Ri->cols && (tracker->stats.obs_count % tracker->report_covariance_cnt) == 0) {
            char prefix[64];
            snprintf(prefix, sizeof(prefix), "%s' FULL_COVARIANCE ", so->codename);
            survive_recording_write_matrix_record(ctx->recptr, prefix, &R);

            if(tracker->report_sampled_cloud > 0) {
                survive_show_covariance(so, pose, Ri, .05, tracker->report_sampled_cloud);
            }

        }

        FLT obs_error = integrate_pose(tracker, time, pose, tracker->obs_model.adaptive ? 0 : &R);
		if (obs_error >= 0) observe_optical_motion(tracker, time, pose, &R);
		tracker->stats.obs_total_error += obs_error;
		tracker->stats.obs_count++;

		SurviveObject *so = tracker->so;
		SV_DATA_LOG("res_err_obs", &obs_error, 1);

		survive_kalman_tracker_report_state(pd, tracker);
	}
}

typedef void (*survive_attach_detach_fn)(SurviveContext *ctx, const char *tag, FLT *var);

void survive_kalman_tracker_reinit(SurviveKalmanTracker *tracker) {
	memset(&tracker->stats, 0, sizeof(tracker->stats));

	tracker->report_ignore_start_cnt = 0;
	tracker->last_light_time = 0;
	tracker->last_unreliable_pose_time = 0;
	tracker->optical_motion_count = tracker->optical_motion_next = 0;
	tracker->optical_motion_until[0] = tracker->optical_motion_until[1] = 0;
	tracker->optical_motion_fit_time = 0;
	tracker->optical_motion_start_time = 0;
	tracker->light_residuals_all = 0;
	tracker->gyro_control_started = false;
	tracker->gyro_control_time = 0;

	memset(&tracker->state, 0, sizeof(tracker->state));
	tracker->state.Pose.Rot[0] = 1;
	tracker->state.IMUBias.IMUCorrection[0] = 1;
	for(int i = 0;i < 3;i++)
		tracker->state.IMUBias.AccScale[i] = 1.;

	cnkalman_state_reset(&tracker->model);
	for (int i = 0; i < 6; i++) {
		cnMatrixSet(&tracker->model.P, i, i, cnMatrixGet(&tracker->model.P, i, i) + 1e5);
	}

	if (tracker->imu_bias_model.state_cnt)
		cnkalman_state_reset(&tracker->imu_bias_model);

	SurviveIMUBiasErrorModel initial_variance = {
		.AccScale = {tracker->params.initial_acc_scale_variance, tracker->params.initial_acc_scale_variance, tracker->params.initial_acc_scale_variance},
		.GyroBias = {
			tracker->params.initial_gyro_variance, tracker->params.initial_gyro_variance, tracker->params.initial_gyro_variance
		},
		.AccBias = {
			tracker->params.initial_acc_bias_variance, tracker->params.initial_acc_bias_variance, tracker->params.initial_acc_bias_variance
		},
		.IMUCorrection = {
			tracker->params.initial_variance_imu_correction, tracker->params.initial_variance_imu_correction, tracker->params.initial_variance_imu_correction
		}
	};
	CnMat bias_covariance = survive_kalman_tracker_bias_covariance(tracker);
	cn_set_diag(&bias_covariance, (const FLT *)&initial_variance);

	size_t state_cnt = tracker->model.state_cnt;

	FLT Rrs = tracker->obs_rot_var;
	FLT Rps = tracker->obs_pos_var;
	FLT Rr[] = {Rrs, Rrs, Rrs, Rrs, Rps, Rps, Rps};
	struct CnMat ObsR = cnMat(7, 7, tracker->Obs_R);
	cn_set_diag(&ObsR, Rr);
	if (tracker->obs_local_rotation) {
		FLT tangent_variance[] = {Rps, Rps, Rps, 4 * Rrs, 4 * Rrs, 4 * Rrs};
		ObsR = cnMat(6, 6, tracker->Obs_R);
		cn_set_diag(&ObsR, tangent_variance);
	}

	FLT Rimu[] = {tracker->acc_var,	 tracker->acc_var,	tracker->acc_var,
				  tracker->gyro_var, tracker->gyro_var, tracker->gyro_var};
	struct CnMat IMU_R = cnMat(6, 6, tracker->IMU_R);
	cn_set_diag(&IMU_R, Rimu);

	FLT var_diag[SURVIVE_MODEL_MAX_STATE_CNT] = {0};
	FLT p_threshold = survive_kalman_tracker_position_var2(tracker, var_diag, tracker->model.error_state_size);
	SurviveObject * so = tracker->so;
	SV_DATA_LOG("tracker_P", var_diag, tracker->model.state_cnt);
}

void survive_kalman_tracker_init(SurviveKalmanTracker *tracker, SurviveObject *so) {
	memset(tracker, 0, sizeof(*tracker));

	tracker->so = so;

	struct SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(110, "Initializing Filter:");
	// These are relatively high numbers to seed with; we are essentially saying
	// origin has a variance of 10m; and the quat can be varied by 4 -- which is
	// more than any actual normalized quat could be off by.

	SurviveKalmanTracker_attach_config(tracker->so->ctx, tracker);
	if (tracker->obs_axisangle_model)
		tracker->obs_local_rotation = false;

	bool use_imu = (bool)survive_configi(ctx, "use-imu", SC_GET, 1);
	if (!use_imu) {
		tracker->gyro_var = tracker->acc_var = -1;
	}

	bool use_kalman = (bool)survive_configi(ctx, "use-kalman", SC_GET, 1);
	tracker->use_raw_obs = tracker->use_raw_obs || !use_kalman;
	// The current gyro-control model uses the continuous kinematic noise
	// model and a fixed, already calibrated IMU frame. Retain the existing
	// measurement filter for explicitly selected legacy/adaptive models.
	tracker->gyro_control_mode = tracker->use_gyro_control && use_imu && tracker->use_error_state &&
		!tracker->use_raw_obs && tracker->noise_model == 0 && tracker->gyro_var >= 0 &&
		tracker->params.initial_variance_imu_correction == 0 &&
		!survive_configi(ctx, "kalman-obj-imu-adaptive", SC_GET, 0) &&
		survive_configi(ctx, "kalman-obj-imu-error-state-model", SC_GET, 1);

	size_t imu_state_cnt = (sizeof(SurviveKalmanModel) - offsetof(SurviveKalmanModel, IMUBias.AccScale)) / sizeof(FLT);
	size_t state_cnt = (sizeof(SurviveKalmanModel) ) / sizeof(FLT) - imu_state_cnt;

	bool joint_bias = use_imu && tracker->use_error_state &&
		(tracker->gyro_control_mode || survive_configi(ctx, KALMAN_JOINT_IMU_BIAS_TAG, SC_GET, 0));
	switch(joint_bias ? 0 : tracker->minimize_state_space) {
	    case 1:
			if(tracker->params.process_weight_gyro_bias) break;
		imu_state_cnt -= 3;
            if(tracker->params.process_weight_acc_bias) break;
		imu_state_cnt -= 3;
            if(tracker->params.initial_variance_imu_correction) break;
		imu_state_cnt -= 4;
            if(tracker->params.initial_acc_scale_variance) break;
		imu_state_cnt -= 1;
            if(tracker->params.process_weight_acc || tracker->params.process_weight_jerk) break;
            state_cnt -= 3;
            if(tracker->params.process_weight_ang_velocity) break;
            state_cnt -= 3;
            if(tracker->params.process_weight_vel) break;
            state_cnt -= 3;
            break;
	    default:
	        break;
	}

	if (joint_bias) {
		state_cnt += imu_state_cnt;
		imu_state_cnt = 0;
	}
	if(imu_state_cnt) {
		cnkalman_error_state_init(&tracker->imu_bias_model, imu_state_cnt, imu_state_cnt - 1, 0, imu_bias_process_noise,
								  imu_error_state_fn, &tracker->params, (FLT*)&tracker->state.IMUBias);
		tracker->imu_bias_model.Update_fn = imu_bias_update_fn;
	}

	if(tracker->use_error_state) {
		cnkalman_error_state_init(&tracker->model, state_cnt, state_cnt - (joint_bias ? 2 : 1), survive_kalman_tracker_predict_jac,
								  tracker->noise_model == 0 ? survive_kalman_tracker_error_process_noise_bounce : 0,
								  error_state_fn, &tracker->params, (FLT *)&tracker->state);
		tracker->model.Update_fn = state_update_fn;
		// Project the full-state transition into error coordinates about the
		// predicted nominal pose. The generated error prediction instead
		// differentiates relative to the old nominal pose, so finite rotation
		// changes even a constant local orientation error's covariance.
		tracker->model.error_state_transition = false;
		if (tracker->gyro_control_mode) {
			tracker->model.user = tracker;
			tracker->model.Q_fn = gyro_control_process_noise;
		}
	} else {
		cnkalman_state_init(&tracker->model, state_cnt, survive_kalman_tracker_predict_jac,
								  tracker->noise_model == 0 ? survive_kalman_tracker_process_noise_bounce : 0,
								  &tracker->params, (FLT *)&tracker->state);
	}
	if(tracker->noise_model == 1) {
		if(tracker->use_error_state) {
			SurviveKalmanErrorModel* pv = (SurviveKalmanErrorModel *)&tracker->process_variance;
			for(int i = 0;i < 3;i++) {
					pv->Pose.Pos[i] = tracker->params.process_weight_pos;
					pv->Pose.AxisAngleRot[i] = tracker->params.process_weight_rotation;
					pv->Velocity.Pos[i] = tracker->params.process_weight_vel;
					pv->Velocity.AxisAngleRot[i] = tracker->params.process_weight_ang_velocity;
					pv->Acc[i] = tracker->params.process_weight_acc;
					pv->IMUBias.AccBias[i] = tracker->params.process_weight_acc_bias;
					pv->IMUBias.GyroBias[i] = tracker->params.process_weight_gyro_bias;
				}
		} else {
			SurviveKalmanModel* pv = &tracker->process_variance;
			for(int i = 0;i < 3;i++) {
				pv->Pose.Pos[i] = tracker->params.process_weight_pos;
				pv->Pose.Rot[i] = tracker->params.process_weight_rotation;
				pv->Velocity.Pos[i] = tracker->params.process_weight_vel;
				pv->Velocity.AxisAngleRot[i] = tracker->params.process_weight_ang_velocity;
				pv->Acc[i] = tracker->params.process_weight_acc;
				pv->IMUBias.AccBias[i] = tracker->params.process_weight_acc_bias;
				pv->IMUBias.GyroBias[i] = tracker->params.process_weight_gyro_bias;
			}
			pv->Pose.Rot[3] = tracker->params.process_weight_rotation;
		}
		tracker->model.state_variance_per_second = cnVec(tracker->model.error_state_size, tracker->process_variance.Pose.Pos);
		if (joint_bias) {
			SurviveKalmanErrorModel *pv = (SurviveKalmanErrorModel *)&tracker->process_variance;
			memset(&pv->IMUBias, 0, sizeof(pv->IMUBias));
			tracker->model.Q_fn = imu_bias_process_noise;
		}
	}
	//tracker->model.transition_jacobian_mode = cnkalman_jacobian_mode_debug;
	if (ctx) {
		cnkalman_set_logging_level(&tracker->model, ctx->log_level);
	}
    tracker->model.normalize_fn = kalman_model_normalize;

	tracker->model.datalog_user = tracker;
	tracker->model.datalog = tracker_datalog;

	cnkalman_state_t* states[] = {0, 0, &tracker->model};
	cnkalman_meas_model_multi_init(states, 3, "joint lightcap", &tracker->joint_model, map_joint_light_data);
	cnkalman_meas_model_t_joint_lightcap_attach_config(ctx, &tracker->joint_model);
	tracker->joint_model.error_state_model = true;
	if(tracker->joint_model.term_criteria.max_iterations == -1)
		tracker->joint_model.term_criteria.max_iterations = 10;

	cnkalman_state_t* models[] = { &tracker->model, &tracker->imu_bias_model };
    cnkalman_meas_model_multi_init(models, joint_bias ? 1 : 2, "imu", &tracker->imu_model, survive_kalman_tracker_imu_measurement_model);
	cnkalman_meas_model_t_obj_imu_attach_config(ctx, &tracker->imu_model);

	cnkalman_meas_model_multi_init(models, joint_bias ? 1 : 2, "zvu", &tracker->zvu_model, 0);

    cnkalman_meas_model_init(&tracker->model, "lightcap", &tracker->lightcap_model, map_light_data);
	cnkalman_meas_model_t_obj_lightcap_attach_config(ctx, &tracker->lightcap_model);
	//tracker->lightcap_model.error_state_model = false;
	tracker->lightcap_model.term_criteria.max_iterations = 10;

    cnkalman_meas_model_init(&tracker->model, "obs", &tracker->obs_model, tracker->obs_local_rotation ?
		map_obs_data_local_rotation : tracker->obs_axisangle_model ? map_obs_data_axisangle : map_obs_data);
	cnkalman_meas_model_t_obj_obs_attach_config(ctx, &tracker->obs_model);
	tracker->obs_model.term_criteria.max_iterations = 10;

	survive_kalman_tracker_reinit(tracker);

	SV_VERBOSE(10, "Tracker config for %s (%d state count)", survive_colorize_codename(tracker->so), (int)state_cnt);
}

SurviveVelocity survive_kalman_tracker_velocity(const SurviveKalmanTracker *tracker) {
	SurviveKalmanModel mdl = {0};
	struct CnMat x1 = cnVec(13, (FLT *)&mdl);
	cnkalman_extrapolate_state(0, &tracker->model, &x1, 0);
	return mdl.Velocity;
}

static void print_kalman_stats(SurviveContext* ctx, const cnkalman_meas_model_t * model) {
    const struct cnkalman_update_extended_total_stats_t* total_stats = &model->stats;
    if(total_stats->total_runs == 0) return;

    SV_VERBOSE(5, "\t%s Kalman statistics:", model->name);
    FLT t = (FLT)total_stats->total_runs;
    SV_VERBOSE(5, "\t\t%-32s %6d %7.3f%%", "failures", total_stats->total_failures, 100 * total_stats->total_failures / t);
    SV_VERBOSE(5, "\t\t%-32s %7.7f / %7.7f / %7.7f", "avg bestnorm", total_stats->bestnorm_acc / t, total_stats->bestnorm_meas_acc / t, total_stats->bestnorm_delta_acc / t);
    SV_VERBOSE(5, "\t\t%-32s %7.7f / %7.7f", "avg orignorm", total_stats->orignorm_acc / t, total_stats->orignorm_meas_acc / t);
    SV_VERBOSE(5, "\t\t%-32s %7.7f", "avg step", total_stats->step_acc / (FLT)total_stats->step_cnt);
    SV_VERBOSE(5, "\t\t%-32s %6d (%3.2f)", "iterations", total_stats->total_iterations, total_stats->total_iterations / (FLT)total_stats->total_runs);
    SV_VERBOSE(5, "\t\t%-32s %6d", "runs", total_stats->total_runs);
    SV_VERBOSE(5, "\t\t%-32s %6d / %6d", "fevals", total_stats->total_fevals, total_stats->total_hevals);
    SV_VERBOSE(5, "\t\t%-32s", "exit reasons");
    for(int i = 1;i < cnkalman_update_extended_termination_reason_MAX;i++) {
        SV_VERBOSE(5, "\t\t    %-28s %6u", cnkalman_update_extended_termination_reason_to_str(i), (int)total_stats->stop_reason_counts[i]);
    }
}

void survive_kalman_tracker_stats(SurviveKalmanTracker *tracker) {
	FLT report_runtime = tracker->last_report_time - tracker->first_report_time;
	FLT imu_runtime = tracker->last_imu_time - tracker->first_imu_time;
    SurviveContext *ctx = tracker->so->ctx;

	SV_VERBOSE(5, "IMU %s tracker statistics:", tracker->so->codename);
	SV_VERBOSE(5, "\t%-32s %4u %4u", "state_cnt", tracker->model.state_cnt, tracker->model.error_state_size);

	LinmathQuat q;
	quatnormalize(q, tracker->state.IMUBias.IMUCorrection);
	SV_VERBOSE(5, "\t%-32s (" Point3_format ") " Point4_format, "IMU Correction", LINMATH_VEC3_EXPAND(tracker->state.IMUBias.AccScale), LINMATH_VEC4_EXPAND(q));
	SV_VERBOSE(5, "\t%-32s " Point3_format, "Acc Bias", LINMATH_VEC3_EXPAND(tracker->state.IMUBias.AccBias));
	SV_VERBOSE(5, "\t%-32s " Point3_format, "Gyro Bias", LINMATH_VEC3_EXPAND(tracker->state.IMUBias.GyroBias));

	SV_VERBOSE(5, "\t%-32s %f", "avg hz", tracker->stats.reported_poses / report_runtime);

	SV_VERBOSE(5, "\t%-32s %u", "late imu", tracker->stats.late_imu_dropped);
	SV_VERBOSE(5, "\t%-32s %u", "late light", tracker->stats.late_light_dropped);
	//joint_model_sensor_cnt_sum
	SV_VERBOSE(5, "\t%-32s %7.7f avg cnt %8d dropped", "joint model", tracker->stats.joint_model_sensor_cnt_sum / (FLT) tracker->joint_model.stats.total_runs,
			   tracker->stats.joint_model_dropped);
	SV_VERBOSE(5, "\t%-32s %7.7f avg cnt %8d dropped", "lightcap model", tracker->stats.lightcap_model_sensor_cnt_sum / (FLT) tracker->lightcap_model.stats.total_runs,
			   tracker->stats.lightcap_model_dropped);

	SV_VERBOSE(5, "\t%-32s %u of %u (%2.2f%%)", "Dropped poses", (unsigned)tracker->stats.dropped_poses,
			   (unsigned)(tracker->stats.reported_poses + tracker->stats.dropped_poses),
			   100. * tracker->stats.dropped_poses /
				   (FLT)(tracker->stats.reported_poses + tracker->stats.dropped_poses))

	FLT var[SURVIVE_MODEL_MAX_STATE_CNT] = {0};
	scalend(var, tracker->stats.reported_var, 1. / tracker->stats.reported_poses, SURVIVE_MODEL_MAX_STATE_CNT);
	SV_VERBOSE(5, "\t%-32s " Point19_format, "Mean reported variance", LINMATH_VEC19_EXPAND(var));
	scalend(var, tracker->stats.dropped_var, 1. / tracker->stats.reported_poses, SURVIVE_MODEL_MAX_STATE_CNT);
	SV_VERBOSE(5, "\t%-32s " Point19_format, "Mean dropped variance", LINMATH_VEC19_EXPAND(var));
	FLT integration_variance[16] = {0};
	variance_tracker_calc(&tracker->pose_variance, integration_variance);
	SV_VERBOSE(5, "\t%-32s %e (%7u integrations, %7.3fhz) " Point7_format, "Obs error",
			   tracker->stats.obs_total_error / (FLT)tracker->stats.obs_count, (unsigned)tracker->stats.obs_count,
			   (unsigned)tracker->stats.obs_count / report_runtime, LINMATH_VEC7_EXPAND(integration_variance));

	variance_tracker_calc(&tracker->imu_variance, integration_variance);
	SV_VERBOSE(5, "\t%-32s %e (%7u integrations, %7.3fhz) " Point6_format, "IMU error",
			   tracker->stats.imu_total_error / (FLT)tracker->stats.imu_count, (unsigned)tracker->stats.imu_count,
			   (unsigned)tracker->stats.imu_count / imu_runtime, LINMATH_VEC6_EXPAND(integration_variance));
	SV_VERBOSE(5, "\t%-32s " FLT_format " " FLT_format " (%7u)", "IMU acc avg norm",
		   tracker->stats.acc_norm / (FLT)tracker->stats.imu_count,  (FLT)tracker->stats.imu_count / tracker->stats.acc_norm,
			   (unsigned)tracker->stats.imu_count);
    SV_VERBOSE(5, "\t%-32s " FLT_format " " FLT_format " (%7u)", "Stationary IMU acc avg norm",
               tracker->stats.stationary_acc_norm / (FLT)tracker->stats.stationary_imu_count,  (FLT)tracker->stats.stationary_imu_count / tracker->stats.stationary_acc_norm,
               (unsigned)tracker->stats.stationary_imu_count);
	SV_VERBOSE(5, "\t%-32s  %7u", "No light IMU count",
			   (unsigned)tracker->stats.no_light_imu_count);

	var[0] = 0;
	for(int lh = 0;lh < NUM_GEN2_LIGHTHOUSES;lh++) {
		for(int sidx = 0; sidx < tracker->so->sensor_ct;sidx++) {
			for(int axis = 0;axis < 2;axis++) {
				if((FLT)tracker->light_variance[lh][sidx][axis].counts) {
					var[0] += tracker->light_variance[lh][sidx][axis].variances[0] /
							  (FLT)tracker->light_variance[lh][sidx][axis].counts;
				}
			}
		}
	}
	SV_VERBOSE(5, "\t%-32s %e (%7u integrations, %7.3fhz) " FLT_format, "Lightcap error",
			   tracker->stats.lightcap_total_error / (FLT)tracker->stats.lightcap_count,
			   (unsigned)tracker->stats.lightcap_count, (unsigned)tracker->stats.lightcap_count / report_runtime, var[0]);

	SV_VERBOSE(5, " ");
	SV_VERBOSE(5, "\t%-32s " Point3_format, "gyro bias", LINMATH_VEC3_EXPAND(tracker->state.IMUBias.GyroBias));
    SV_VERBOSE(5, "\t%-32s " FLT_format, "Lightcap R", tracker->light_var);
	for (int i = 0; i < 6; i++) {
		SV_VERBOSE(5, "\t%-32s " Point6_format, i == 0 ? "Gyro R" : "", LINMATH_VEC6_EXPAND(tracker->IMU_R + 6 * i));
	}
	if (tracker->obs_local_rotation) {
		for (int i = 0; i < 6; i++)
			SV_VERBOSE(5, "\t%-32s " Point6_format, i == 0 ? "Observation R" : "",
					   LINMATH_VEC6_EXPAND(tracker->Obs_R + 6 * i));
	} else {
		for (int i = 0; i < 7; i++)
			SV_VERBOSE(5, "\t%-32s " Point7_format, i == 0 ? "Observation R" : "",
					   LINMATH_VEC7_EXPAND(tracker->Obs_R + 7 * i));
	}

	FLT* state_variance = (FLT*)&tracker->reported_state_variance;
    scalend(state_variance, state_variance, 1. / (FLT)tracker->state_variance_count, tracker->model.state_cnt);
    SV_VERBOSE(5, "\t%-32s " Point26_format, "Observed state variance", LINMATH_VEC26_EXPAND(state_variance));

    for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (tracker->stats.lightcap_count_by_lh[i]) {
			SV_VERBOSE(5, "\tLighthouse %d", i);
			SV_VERBOSE(5, "\t\t%-32s %e", "Avg error",
					   tracker->stats.lightcap_error_by_lh[i] / tracker->stats.lightcap_count_by_lh[i]);
			SV_VERBOSE(5, "\t\t%-32s %u", "Count", (unsigned)tracker->stats.lightcap_count_by_lh[i]);
			SV_VERBOSE(5, "\t\t%-32s %e", "Current error", tracker->light_residuals[i]);
		}

		for (int j = 0; j < SENSORS_PER_OBJECT; j++) {
			for (int z = 0; z < 2; z++) {
				if (tracker->so->activations.hits[j][i][z]) {
					SV_VERBOSE(5, "\t\t %02d.%d %5d %f", j, z, (int)tracker->so->activations.hits[j][i][z],
							   tracker->so->activations.hits[j][i][z] / report_runtime);
				}
			}
		}
	}
    print_kalman_stats(ctx, &tracker->imu_model);
    print_kalman_stats(ctx, &tracker->lightcap_model);
    print_kalman_stats(ctx, &tracker->obs_model);
	print_kalman_stats(ctx, &tracker->zvu_model);
	print_kalman_stats(ctx, &tracker->joint_model);

	memset(&tracker->stats, 0, sizeof(tracker->stats));
	tracker->first_report_time = tracker->last_report_time = 0;

	SV_VERBOSE(5, " ");
}
void survive_kalman_tracker_free(SurviveKalmanTracker *tracker) {
	SurviveContext *ctx = tracker->so->ctx;

	survive_kalman_tracker_stats(tracker);

	cnkalman_state_free(&tracker->model);
	cnkalman_state_free(&tracker->imu_bias_model);

	cnkalman_meas_model_t_joint_lightcap_detach_config(tracker->so->ctx, &tracker->joint_model);
	cnkalman_meas_model_t_obj_imu_detach_config(tracker->so->ctx, &tracker->imu_model);
	cnkalman_meas_model_t_obj_obs_detach_config(tracker->so->ctx, &tracker->obs_model);
	cnkalman_meas_model_t_obj_lightcap_detach_config(tracker->so->ctx, &tracker->lightcap_model);

	SurviveKalmanTracker_detach_config(tracker->so->ctx, tracker);
}

void survive_kalman_tracker_lost_tracking(SurviveKalmanTracker *tracker, bool allowLHReset) {
	if (tracker == 0)
		return;

	SurviveContext *ctx = tracker->so->ctx;
	SV_WARN("Too many failures for %s at %f; reseting calibration %e (%7.4f stationary)", survive_colorize_codename(tracker->so),
			survive_run_time(ctx),
			tracker->light_residuals_all,
			SurviveSensorActivations_stationary_time(&tracker->so->activations) / 48000000.);
	tracker->light_residuals_all = 0;
	{
		tracker->so->OutPoseIMU = (SurvivePose){0};
		tracker->so->poseConfidence = 0;
		survive_kalman_tracker_reinit(tracker);
		memset(&tracker->so->OutPoseIMU, 0, sizeof(SurvivePose));
		memset(&tracker->so->OutPose, 0, sizeof(SurvivePose));
	}

	if (!allowLHReset)
		return;

	bool objectsAreValid = false;
	for (int i = 0; i < ctx->objs_ct && !objectsAreValid; i++) {
		objectsAreValid |= !quatiszero(ctx->objs[i]->OutPoseIMU.Rot);
	}

	if (!objectsAreValid) {
	  ctx->floor_offset = 0;
		for (int lh = 0; lh < ctx->activeLighthouses; lh++) {
			ctx->bsd[lh].PositionSet = 0;
			SV_WARN("Lost tracking for LH%d %f", lh, tracker->light_residuals[lh]);
		}
	}
}

bool survive_kalman_tracker_check_valid(SurviveKalmanTracker *tracker) {
	bool isValid =
		tracker->light_error_threshold <= 0 || tracker->light_residuals_all < tracker->light_error_threshold ||
		(SurviveSensorActivations_stationary_time(&tracker->so->activations) < tracker->so->timebase_hz / 10);

	for (int i = 0; i < 3; i++) {
		isValid &= fabsf((float)tracker->state.Pose.Pos[i]) < 20;
	}

	if (!isValid) {
		survive_kalman_tracker_lost_tracking(tracker, tracker->use_error_for_lh_pos);
		return false;
	}
	return true;
}

void survive_kalman_tracker_report_state(PoserData *pd, SurviveKalmanTracker *tracker) {
	SurvivePose pose = {0};
	normalize_model(tracker);

	FLT t = pd->timecode / (FLT)tracker->so->timebase_hz;

	if (t < tracker->model.t) {
		assert(tracker->model.t - t < 1);
		t = tracker->model.t;
	}

	SurviveObject *so = tracker->so;
	assert(fabs(1 - quatmagnitude(tracker->state.Pose.Rot)) < 1e-4);
	SV_DATA_LOG("model_state", (const FLT *)&tracker->state, tracker->model.state_cnt);

	if (tracker->so->conf == 0) {
		return;
	}

	SurviveContext *ctx = tracker->so->ctx;
	if (tracker->stats.reported_poses) {
		if (pd->timecode <= tracker->last_report_timecode)
			return;
		double minimum = tracker->min_report_time;
		if (minimum < 0) {
			// Nominal 250 Hz IMUs can run slightly faster (e.g. 3.989 ms).
			// A strict 4 ms minimum drops every other sample. Allow 1% clock
			// variation only for the default, retaining a rate limit for light
			// updates between IMU samples. Explicit limits receive no slack.
			minimum = so->imu_freq > 0 ? .99 / so->imu_freq : 0;
		}
		// Quantize the requested interval to the nearest sensor tick. Avoid
		// subtracting large floating-point times at exact-period boundaries.
		double minimum_ticks = floor(minimum * so->timebase_hz + .5);
		if ((double)(pd->timecode - tracker->last_report_timecode) < minimum_ticks)
			return;
	}

	if (!survive_kalman_tracker_check_valid(tracker)) {
		tracker->stats.dropped_poses++;
		return;
	}

	survive_kalman_tracker_predict(tracker, t, &pose);
	SV_DATA_LOG("model_predict", (FLT *)pose.Pos, sizeof(pose) / sizeof(FLT));

	size_t state_cnt = tracker->model.state_cnt;
	FLT var_diag[SURVIVE_MODEL_MAX_STATE_CNT] = {0};
	FLT p_threshold = survive_kalman_tracker_position_var2(tracker, var_diag, tracker->model.error_state_size);
	SV_DATA_LOG("tracker_P", var_diag, tracker->model.state_cnt);

	if ((tracker->report_threshold_var > 0 && p_threshold >= tracker->report_threshold_var) ||
		(tracker->report_ignore_start > tracker->report_ignore_start_cnt)) {
		tracker->stats.dropped_poses++;
		addnd(tracker->stats.dropped_var, var_diag, tracker->stats.dropped_var, state_cnt);
		tracker->report_ignore_start_cnt++;

		so->OutPoseIMU = pose;
		return;
	}

	addnd(tracker->stats.reported_var, var_diag, tracker->stats.reported_var, state_cnt);

	SV_VERBOSE(600, "Tracker variance %s " Point16_format, tracker->so->codename, LINMATH_VEC16_EXPAND(var_diag));
	SV_VERBOSE(600, "Tracker Bias %s     " Point3_format, tracker->so->codename,
			   LINMATH_VEC3_EXPAND(tracker->state.IMUBias.GyroBias));
	SV_VERBOSE(600, "%f Tracker report %s   " SurvivePose_format, survive_run_time(ctx), tracker->so->codename,
			   SURVIVE_POSE_EXPAND(pose));

	tracker->stats.reported_poses++;

	SurviveVelocity velocity = survive_kalman_tracker_velocity(tracker);

	if (tracker->first_report_time == 0) {
		tracker->first_report_time = t;
	}

	tracker->so->poseConfidence = 1. / p_threshold;

    if(tracker->last_report_time > 0) {
        FLT dt = (pd->timecode - tracker->last_report_timecode) / (FLT)so->timebase_hz;
        struct survive_kalman_model_t diff = { 0 };
        subnd((FLT *) &diff, (FLT *) &tracker->state, (FLT *) &tracker->previous_state, state_cnt);
        scalend((FLT *) &diff, (FLT *) &diff, 1. / dt, state_cnt);
        mulnd((FLT *) &diff, (FLT *) &diff, (FLT *) &diff, state_cnt);
        addnd((FLT *) &tracker->reported_state_variance, (FLT *) &tracker->reported_state_variance, (FLT *) &diff, state_cnt);
        tracker->state_variance_count++;
    }

    tracker->last_report_time = t;
	tracker->last_report_timecode = pd->timecode;

    if(tracker->report_covariance_cnt > 0 && tracker->stats.reported_poses % tracker->report_covariance_cnt == 0) {
		survive_recording_write_to_output(ctx->recptr, "%s FULL_STATE " Point27_format "\n",
                                          so->codename, LINMATH_VEC27_EXPAND((FLT*)&tracker->state));
        char prefix[64];
        snprintf(prefix, sizeof(prefix), "%s FULL_COVARIANCE ", so->codename);
        survive_recording_write_matrix_record(ctx->recptr, prefix, &tracker->model.P);
		CnMat bias_covariance = survive_kalman_tracker_bias_covariance(tracker);
		survive_recording_write_matrix(ctx->recptr, so, 5, "imu", &bias_covariance);

		if(tracker->report_sampled_cloud > 0) {
			survive_show_covariance(so, &pose, &tracker->model.P, .1, tracker->report_sampled_cloud);
		}
		{
			FLT v[16] = { 0 };
			int bias_state_cnt = survive_kalman_tracker_has_joint_bias(tracker) ?
				sizeof(SurviveIMUBiasModel) / sizeof(FLT) : tracker->imu_bias_model.state_cnt;
			memcpy(v, &tracker->state.IMUBias.AccScale, sizeof(FLT) * bias_state_cnt);
			CnMat meta = cnVec(bias_state_cnt, v);
			for(int i = 0;i < 4;i++)
				v[i] = v[i] - 1;

			if(cn_norm2(&meta) != 0) {
				survive_recording_write_matrix(ctx->recptr, so, 10, "meta", &meta);
			}
		}
    }

    tracker->previous_state = tracker->state;
    copy3d(so->acceleration, tracker->state.Acc);
	SV_VERBOSE(110, "%s confidence %7.7f", survive_colorize_codename(so), 1. / p_threshold);
	if (so->OutPose_timecode < pd->timecode) {
		SURVIVE_INVOKE_HOOK_SO(imupose, so, pd->timecode, &pose);
	}
	if(tracker->stats.imu_count > 100) {
        SURVIVE_INVOKE_HOOK_SO(velocity, so, pd->timecode, &velocity);
    }
}
