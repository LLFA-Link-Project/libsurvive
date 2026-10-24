#include "survive.h"

#include "../generated/kalman_kinematics.gen.h"
#include "survive_optimizer.h"
#include "survive_reproject_gen2.h"
#include "test_case.h"
#include "../survive_kalman_tracker.h"
#include "../survive_calibration.h"

survive_optimizer_settings settings = {
	.optimize_scale_threshold = -1,
};

static SurvivePose run(survive_optimizer* mpfitctx, SurviveKalmanModel* mdl, const FLT* points, size_t points_cnt, mp_result* result, CnMat* R, const SurviveVelocity* vel) {
	mpfitctx->ptsLength = points_cnt;
	SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(*mpfitctx, 0);

	quatnormalize(mdl->Pose.Rot, mdl->Pose.Rot);
	SurvivePose lh_pose = {
		.Pos = { 0, 0, -5 },
		.Rot = { 1 }
	};
	SurvivePose ilh = InvertPoseRtn(&lh_pose);

	BaseStationCal bcal[2] = { 0 };

	SurvivePose orig = mdl->Pose;//{ .Rot = { 1 } };
	survive_optimizer_setup_pose(mpfitctx, &orig, false, 1);
	survive_optimizer_setup_camera(mpfitctx, 0, &ilh, true, 1);
	survive_optimizer_parameter * pt_params = survive_optimizer_emplace_params(mpfitctx, survive_optimizer_parameter_obj_points, points_cnt);
	memcpy(pt_params->p, points, points_cnt * sizeof(FLT) * 3);

	survive_optimizer_parameter * bsd_params = survive_optimizer_emplace_params(mpfitctx, survive_optimizer_parameter_camera_parameters, 1);
	memset(bsd_params->p, 0, bsd_params->size * sizeof(FLT));


	if(!mpfitctx->disableVelocity){
		SurviveVelocity *velocity = survive_optimizer_get_velocity(mpfitctx);
		*velocity = vel ? *vel : mdl->Velocity;
	}

	FLT t = 0;
	for(int i = 0;i < 2;i++) {
		t += .1;
		SurviveKalmanModel tmp = {0};
		SurviveKalmanModelPredict(&tmp, .1, mdl);
		*mdl = tmp;
		quatnormalize(mdl->Pose.Rot, mdl->Pose.Rot);
		for(int j = 0;j < points_cnt;j++) {
			survive_optimizer_measurement *meas =
				survive_optimizer_emplace_meas(mpfitctx, survive_optimizer_measurement_type_light);
			meas->variance = 1e-4;
			meas->time = t;
			meas->light.sensor_idx = j;
			meas->light.axis = i & 1;

			FLT out[2];
			survive_reproject_full(bcal, &lh_pose, &mdl->Pose, &points[j*3], out);
			meas->light.value = out[i & 1];
		}
	}
	mpfitctx->timecode = t;
	SurvivePose *opt_pose = survive_optimizer_get_pose(mpfitctx);
	//*opt_pose = mdl.Pose;

	survive_optimizer_run(mpfitctx, result, R);
	return *opt_pose;
}

FLT points[] = {
	-.1, -.1, 0,
	-.1, +.1, 0,
	+.1, -.1, 0,
	+.1, +.1, 0,
	0, 0, .1,
	0, 0, -.1
};


static SurvivePose poseDiff(const SurvivePose *a, const SurvivePose *b) {
	if (quatiszero(a->Rot) && quatiszero(b->Rot)) {
		return (SurvivePose) { 0 };
	}

	SurvivePose iB = InvertPoseRtn(b);
	SurvivePose nearId;
	ApplyPoseToPose(&nearId, a, &iB);
	return nearId;
}

survive_optimizer default_optimizer() {
	survive_optimizer mpfitctx = {
		.settings = &settings,
		.reprojectModel = &survive_reproject_gen1_model,
		.poseLength = 1,
		.cameraLength = 1,
		.objectUpVectorVariance = -1,
		.disableVelocity = true,
		.cfg = survive_optimizer_precise_config(),
		.user = 0
	};
	return mpfitctx;
}

static void count_pose_without_covariance(SurviveObject *so, survive_long_timecode timecode,
										  const SurvivePose *pose) {
	so->OutPoseIMU = *pose;
}

TEST(Optimizer, DispatchPoseWithoutCovariance) {
	SurviveContext ctx = {.imuposeproc = count_pose_without_covariance};
	SurviveObject so = {.ctx = &ctx};
	SurviveKalmanTracker tracker = {.so = &so, .use_raw_obs = true};
	so.tracker = &tracker;
	PoserData pd = {.pt = POSERDATA_SYNC, .timecode = 12345};
	SurvivePose pose = {.Pos = {1, 2, 3}, .Rot = {1}};
	PoserData_poser_pose_func(&pd, &so, &pose, -1, 0);
	ASSERT_EQ(ctx.imupose_call_cnt, 1);
	ASSERT_DOUBLE_ARRAY_EQ(3, so.OutPoseIMU.Pos, pose.Pos);
	ASSERT_QUAT_EQ(so.OutPoseIMU.Rot, pose.Rot);
	return 0;
}

TEST(Optimizer, PairedMeasurementsMustShareObjectAndLighthouse) {
	for (int gen2 = 0; gen2 < 2; gen2++) {
		for (int different_object = 0; different_object < 2; different_object++) {
			survive_optimizer_settings local_settings = {.optimize_scale_threshold = -1, .use_quat_model = true,
													 .disable_filter = true};
			survive_optimizer opt = {.settings = &local_settings, .reprojectModel = gen2 ? &survive_reproject_gen2_model : &survive_reproject_gen1_model,
				.poseLength = 2, .cameraLength = 2, .ptsLength = 1, .disableVelocity = true,
				.objectUpVectorVariance = -1};
			SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(opt, 0);
			SurvivePose objects[2] = {{.Rot = {1}}, {.Pos = {.4, .7, .2}, .Rot = {1}}};
			SurvivePose world2lh_poses[2] = {{.Pos = {0, 0, -5}, .Rot = {1}},
				{.Pos = {-1, -2, -4}, .Rot = {1}}};
			for (int i = 0; i < 2; i++) {
				survive_optimizer_setup_pose_n(&opt, &objects[i], i, true, 1);
				SurvivePose inverse = InvertPoseRtn(&world2lh_poses[i]);
				survive_optimizer_setup_camera(&opt, i, &inverse, true, 1);
			}
			FLT point[3] = {.03, .07, .02};
			survive_optimizer_parameter *pts = survive_optimizer_emplace_params(&opt,
				survive_optimizer_parameter_obj_points, 1);
			memcpy(pts->p, point, sizeof(point));
			survive_optimizer_parameter *cal = survive_optimizer_emplace_params(&opt,
				survive_optimizer_parameter_camera_parameters, 2);
			memset(cal->p, 0, cal->size * sizeof(FLT));
			BaseStationCal bcal[2] = {{.phase = .01, .tilt = .002}, {.phase = -.02, .tilt = -.003}};
			for (int lh = 0; lh < 2; lh++)
				memcpy((BaseStationCal *)cal->p + 2 * lh, bcal, sizeof(bcal));
			for (int axis = 0; axis < 2; axis++) {
				int object = different_object ? axis : 0;
				int lh = different_object ? 0 : axis;
				FLT angles[2];
				LinmathPoint3d world_point, lighthouse_point;
				ApplyPoseToPoint(world_point, &objects[object], point);
				ApplyPoseToPoint(lighthouse_point, &world2lh_poses[lh], world_point);
				opt.reprojectModel->reprojectXY(bcal, lighthouse_point, angles);
				survive_optimizer_measurement *meas = survive_optimizer_emplace_meas(&opt,
					survive_optimizer_measurement_type_light);
				meas->light.axis = axis;
				meas->light.lh = lh;
				meas->light.object = object;
				meas->light.value = angles[axis];
				meas->variance = .01;
			}
			local_settings.disallow_pair_calc = true;
			FLT separate_norm = survive_optimizer_current_norm(&opt);
			ASSERT_DOUBLE_EQ(separate_norm, 0.);
			local_settings.disallow_pair_calc = false;
			ASSERT_DOUBLE_EQ(survive_optimizer_current_norm(&opt), separate_norm);
		}
	}
	return 0;
}

TEST(Optimizer, ReducedChiSquareUsesResidualDegreesOfFreedom) {
	survive_optimizer_settings local_settings = {.optimize_scale_threshold = -1, .use_quat_model = true,
												 .disable_filter = true};
	survive_optimizer opt = {.settings = &local_settings, .reprojectModel = &survive_reproject_gen1_model,
							.poseLength = 1, .cameraLength = 1, .disableVelocity = true, .objectUpVectorVariance = -1,
							.cfg = survive_optimizer_precise_config()};
	SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(opt, 0);
	SurvivePose pose = {.Rot = {1}};
	survive_optimizer_setup_pose(&opt, &pose, true, 1);
	survive_optimizer_setup_camera(&opt, 0, &pose, true, 1);
	opt.mp_parameters_info[0].fixed = false;
	for (int i = 0; i < 4; i++) {
		survive_optimizer_measurement *meas =
			survive_optimizer_emplace_meas(&opt, survive_optimizer_measurement_type_parameters_bias);
		meas->parameter_bias.parameter_index = 0;
		meas->parameter_bias.expected_value = i;
		meas->variance = .1;
	}
	CN_CREATE_STACK_MAT(R, 7, 7);
	mp_result result = {0};
	ASSERT_GT(survive_optimizer_run(&opt, &result, &R), 0);
	ASSERT_DOUBLE_EQ(opt.parameters[0], 1.5);
	ASSERT_EQ(result.nfree, 1);
	ASSERT_EQ(result.nfunc, 4);
	// (J^T J)^-1 = .01/4; sum squared residuals = 500; residual DOF = 3.
	ASSERT_DOUBLE_EQ(cnMatrixGet(&R, 0, 0), (.01 / 4) * result.bestnorm / 3);
	return 0;
}

TEST(Optimizer, GravityResidualDoesNotCancelBetweenAxes) {
	for (int camera = 0; camera < 2; camera++) {
		survive_optimizer_settings local_settings = {.optimize_scale_threshold = -1, .disable_filter = true};
		survive_optimizer opt = {.settings = &local_settings, .reprojectModel = &survive_reproject_gen1_model,
			.poseLength = 1, .cameraLength = 1, .disableVelocity = true, .objectUpVectorVariance = -1};
		SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(opt, 0);
		SurvivePose pose = {.Rot = {1}};
		survive_optimizer_setup_pose(&opt, &pose, true, 1);
		survive_optimizer_setup_camera(&opt, 0, &pose, true, 1);
		const FLT up[] = {0, 0, 1};
		if (camera)
			survive_optimizer_set_cam_up_vector(&opt, 0, .1, up);
		else
			survive_optimizer_set_obj_up_vector(&opt, 0, .1, up);
		// current_norm evaluates the optimizer's axis-angle representation.
		SurvivePose *rotated = camera ? survive_optimizer_get_camera(&opt) : survive_optimizer_get_pose(&opt);
		memset(rotated->Rot, 0, sizeof(rotated->Rot));
		rotated->Rot[1] = LINMATHPI / 2;
		// R_y(90) * up - up = {1, 0, -1}. Its sum is zero, but its squared norm is two.
		ASSERT_DOUBLE_EQ(survive_optimizer_current_norm(&opt), 200.);
		ASSERT_DOUBLE_EQ(opt.stats.object_up_error, 2.);
		ASSERT_EQ(opt.stats.object_up_error_cnt, 1);
		opt.measurements[0].invalid = true;
		ASSERT_DOUBLE_EQ(survive_optimizer_current_norm(&opt), 0.);
	}
	return 0;
}

TEST(Optimizer, CalibrationGravityIsIndependentOfMountingAngle) {
	const FLT world_up[] = {0, 0, 1};
	// Include upright, inverted and steeply tilted installations. The expected
	// accelerometer is world gravity expressed in each base station's frame.
	for (int degrees = 0; degrees <= 180; degrees += 15) {
		FLT euler[] = {degrees * LINMATHPI / 180, .37, -.81};
		SurvivePose pose = {0};
		quatfromeuler(pose.Rot, euler);
		LinmathQuat inverse;
		quatgetconjugate(inverse, pose.Rot);
		LinmathPoint3d local_up;
		quatrotatevector(local_up, inverse, world_up);
		scalend(local_up, local_up, 127, 3);
		ASSERT_DOUBLE_EQ(survive_calibration_up_error(&pose, local_up), 0.);
		scalend(local_up, local_up, -1, 3);
		ASSERT_DOUBLE_EQ(survive_calibration_up_error(&pose, local_up), LINMATHPI);
	}
	SurvivePose identity = {.Rot = {1}};
	const FLT missing[] = {0, 0, 0};
	ASSERT_EQ(isnan(survive_calibration_up_error(&identity, missing)), true);
	identity.Rot[0] = NAN;
	ASSERT_EQ(isnan(survive_calibration_up_error(&identity, world_up)), true);
	return 0;
}

TEST(Optimizer, CameraCovarianceAfterFixedObject) {
	survive_optimizer_settings local_settings = {.optimize_scale_threshold = -1, .disable_filter = true};
	survive_optimizer opt = {.settings = &local_settings, .reprojectModel = &survive_reproject_gen1_model,
							.poseLength = 1, .cameraLength = 1, .disableVelocity = true,
							.objectUpVectorVariance = -1, .dontScaleCov = true,
							.cfg = survive_optimizer_precise_config()};
	SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(opt, 0);
	SurvivePose object = {.Rot = {.5, .5, .5, .5}};
	SurvivePose camera = {.Rot = {1}};
	survive_optimizer_setup_pose(&opt, &object, true, 1);
	survive_optimizer_setup_camera(&opt, 0, &camera, false, 1);
	int camera_index = survive_optimizer_get_camera_index(&opt);
	// Independent zero-valued constraints on the camera translation/axis angle.
	for (int repeat = 0; repeat < 2; repeat++) {
		for (int i = 0; i < 6; i++) {
			survive_optimizer_measurement *meas =
				survive_optimizer_emplace_meas(&opt, survive_optimizer_measurement_type_parameters_bias);
			meas->parameter_bias.parameter_index = camera_index + i;
			meas->parameter_bias.expected_value = 0;
			meas->variance = .1;
		}
	}
	CN_CREATE_STACK_MAT(R, 14, 14);
	mp_result result = {0};
	ASSERT_GT(survive_optimizer_run(&opt, &result, &R), 0);
	ASSERT_EQ(R.rows, 7);
	// AA covariance is .1^2/2. At identity dq_xyz/dAA = .5 I, dq_w/dAA = 0.
	FLT expected[] = {.005, .005, .005, 0, .00125, .00125, .00125};
	for (int i = 0; i < 7; i++) {
		ASSERT_DOUBLE_EQ(cnMatrixGet(&R, i, i), expected[i]);
	}
	return 0;
}

bool verify_R(CnMat* R, SurvivePose* gt, SurvivePose* modeled) {
	FLT v[7];
	cn_get_diag(R, v, 7);
	for(int i = 0;i < 7;i++) v[i] = sqrt(v[i]);

	SurvivePose d = poseDiff(gt, modeled);
	d.Rot[0] = fabs(1 - fabs(d.Rot[0]));
	FLT* dd = &d.Pos[0];
	for(int i = 0;i < 7;i++) {
		if (fabs(dd[i]) > (10 * v[i] + 1e-6))
			return false;
	}

	return true;
}

TEST(Optimizer, Simple) {
	survive_optimizer mpfitctx = default_optimizer();

	SurviveKalmanModel mdl = {.Pose = {.Rot = {1, 1, 1, 1}},
							  .Velocity = {.Pos = {0, 0, .1}, .AxisAngleRot = {0, 0, .1}},
							  .IMUBias = {
								  .IMUCorrection = {1},
								  .AccScale = 1,
							  }};
	CN_CREATE_STACK_MAT(R, 7, 7);

	mp_result results = {0};
	SurvivePose output = run(&mpfitctx, &mdl, points, SURVIVE_ARRAY_SIZE(points) / 3, &results, &R, 0);

	assert(results.bestnorm < 1e1);
	assert(verify_R(&R, &mdl.Pose, &output));

	return  0;
}

TEST(Optimizer, Velocity) {
	survive_optimizer mpfitctx = default_optimizer();
	mpfitctx.disableVelocity = false;

	SurviveKalmanModel mdl = {.Pose = {.Rot = {1, 1, 1, 1}},
							  .Velocity = {.Pos = {0, 0, .1}, .AxisAngleRot = {0, 0, .1}},
							  .IMUBias = {
								  .IMUCorrection = {1},
								  .AccScale = 1,
							  }};
	CN_CREATE_STACK_MAT(R, 7, 7);

	mp_result results = {0};
	SurvivePose output = run(&mpfitctx, &mdl, points, SURVIVE_ARRAY_SIZE(points) / 3, &results, &R, 0);
	assert(results.bestnorm < 1e-5);

	assert(verify_R(&R, &mdl.Pose, &output));
	return  0;
}


FLT side_points[] = {
	-1e-3, -.1, 0,
	-1e-3, +.1, 0,
	+1e-3, -.1, 0,
	+1e-3, +.1, 0,
	0, 0, .1,
	0, 0, 0
};


TEST(Optimizer, SimpleSide) {
	survive_optimizer mpfitctx = default_optimizer();

	SurviveKalmanModel mdl = {.Pose = {.Rot = {1, 1, 1, 1}},
							  .Velocity = {.Pos = {0, 0, .1}, .AxisAngleRot = {0, 0, .1}},
							  .IMUBias = {
								  .IMUCorrection = {1},
								  .AccScale = 1,
							  }};
	CN_CREATE_STACK_MAT(R, 7, 7);

	mp_result results = {0};
	SurvivePose output = run(&mpfitctx, &mdl, side_points, SURVIVE_ARRAY_SIZE(side_points) / 3, &results, &R, 0);

	//assert(verify_R(&R, &mdl.Pose, &output));

	return  0;
}


TEST(Optimizer, VelocitySide) {
	survive_optimizer mpfitctx = default_optimizer();
	mpfitctx.disableVelocity = false;

	SurviveKalmanModel mdl = {.Pose = {.Rot = {1, 1, 1, 1}},
							  .Velocity = {.Pos = {0, 0, .1}, .AxisAngleRot = {0, 0, .1}},
							  .IMUBias = {
								  .IMUCorrection = {1},
								  .AccScale = 1,
							  }};
	CN_CREATE_STACK_MAT(R, 7, 7);

	SurviveVelocity velocity = { .Pos = { 0, 0, -.1 }, .AxisAngleRot = { 0, 0, -.1 } };
	mp_result results = {0};
	SurvivePose output = run(&mpfitctx, &mdl, side_points, SURVIVE_ARRAY_SIZE(side_points) / 3, &results, &R, &velocity);
	SurviveVelocity v1 = *survive_optimizer_get_velocity(&mpfitctx);
	// assert(verify_R(&R, &mdl.Pose, &output));

	return 0;
}

FLT under_specified[] = {
	-.1, -.5, 0, +.1, -.5, 0, 0, .5, 0,
};

TEST(Optimizer, SimpleUnderspecced) {
	survive_optimizer mpfitctx = default_optimizer();

	SurviveKalmanModel mdl = {.Pose = {.Rot = {1, 0, 1, 0}},
							  //.Velocity = { .Pos = { 0, 0, .1}, .AxisAngleRot = {0, 0, .1}},
							  .IMUBias = {
								  .IMUCorrection = {1},
								  .AccScale = 1,
							  }};
	CN_CREATE_STACK_MAT(R, 7, 7);

	mp_result results = {0};
	SurvivePose output =
		run(&mpfitctx, &mdl, under_specified, SURVIVE_ARRAY_SIZE(under_specified) / 3, &results, &R, 0);

	assert(verify_R(&R, &mdl.Pose, &output));

	return  0;
}
