#include "survive.h"

#include "../generated/kalman_kinematics.gen.h"
#include "survive_optimizer.h"
#include "survive_reproject_gen2.h"
#include "test_case.h"

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


TEST(Optimizer, LighthouseResidualsFollowScalarOffsetsAfterGravity) {
	for (int gen2 = 0; gen2 < 2; gen2++) {
		SurviveContext ctx = {0};
		SurviveObject so = {.ctx = &ctx};
		survive_optimizer_settings config = {.optimize_scale_threshold = -1};
		survive_optimizer opt = {.settings = &config, .poseLength = 1, .cameraLength = 3,
			.ptsLength = 6, .disableVelocity = true, .objectUpVectorVariance = -1,
			.cfg = survive_optimizer_precise_config(),
			.reprojectModel = gen2 ? &survive_reproject_gen2_model : &survive_reproject_gen1_model};
		SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(opt, 0);
		opt.sos[0] = &so;
		SurvivePose pose = {.Rot = {1}};
		survive_optimizer_setup_pose(&opt, &pose, true, 1);
		opt.mp_parameters_info[0].fixed = false;
		SurvivePose cameras[3] = {{.Pos = {0, 0, 3}, .Rot = {1}},
			{.Pos = {-1, -.5, 2}, .Rot = {1}}, {.Pos = {1, .5, 2.5}, .Rot = {1}}};
		for (int lh = 0; lh < 3; lh++)survive_optimizer_setup_camera(&opt, lh, &cameras[lh], true, 1);
		survive_optimizer_parameter *pts = survive_optimizer_emplace_params(&opt, survive_optimizer_parameter_obj_points, 6);
		memcpy(pts->p, points, sizeof(points));
		survive_optimizer_parameter *cal = survive_optimizer_emplace_params(&opt, survive_optimizer_parameter_camera_parameters, 3);
		memset(cal->p, 0, cal->size * sizeof(FLT));
		const FLT up[] = {0, 0, 1};
		survive_optimizer_set_obj_up_vector(&opt, 0, .01, up);
		ASSERT_EQ(opt.measurements[0].size, 3);
		BaseStationCal bcal[2] = {0};
		for (int lh = 0; lh < 3; lh++)for (int sensor = 0; sensor < 6; sensor++) {
			SurvivePose inverse = InvertPoseRtn(&cameras[lh]);
			LinmathPoint3d in_lh;
			ApplyPoseToPoint(in_lh, &inverse, points + sensor*3);
			FLT angles[2];
			opt.reprojectModel->reprojectXY(bcal, in_lh, angles);
			for (int axis = 0; axis < 2; axis++) {
				survive_optimizer_measurement *meas = survive_optimizer_emplace_meas(&opt, survive_optimizer_measurement_type_light);
				meas->light.lh = lh;
				meas->light.sensor_idx = sensor;
				meas->light.axis = axis;
				// The last pair from LH0 must not be attributed to LH1.
				meas->light.value = angles[axis]+(lh == 0 && sensor == 5 ? .2 : .000001);
				meas->variance = .0001;
			}
		}
		mp_result result = {0};
		ASSERT_GT(survive_optimizer_run(&opt, &result, 0), 0);
		int counts[3][2] = {{0}};
		for (int i = 0; i < opt.measurementsCnt; i++) {
			const survive_optimizer_measurement *meas = &opt.measurements[i];
			if (meas->meas_type == survive_optimizer_measurement_type_light)
				counts[meas->light.lh][meas->light.axis]++;
		}
		ASSERT_EQ(counts[0][0], 0);
		ASSERT_EQ(counts[0][1], 0);
		ASSERT_EQ(counts[1][0], 6);
		ASSERT_EQ(counts[1][1], 6);
		ASSERT_EQ(counts[2][0], 6);
		ASSERT_EQ(counts[2][1], 6);
	}
	return 0;
}
