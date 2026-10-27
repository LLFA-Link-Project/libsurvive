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

TEST(Optimizer, AntipodalNearIdentitySeedsHaveTheSamePoseAndCovariance) {
	for (int gen2 = 0; gen2 < 2; gen2++) {
		SurvivePose reference = {0};
		FLT reference_covariance[49] = {0};
		for (int sign = 0; sign < 2; sign++) {
			survive_optimizer_settings local_settings = {.optimize_scale_threshold = -1, .disable_filter = true};
			survive_optimizer opt = {.settings = &local_settings,
				.reprojectModel = gen2 ? &survive_reproject_gen2_model : &survive_reproject_gen1_model,
				.poseLength = 1, .cameraLength = 3, .ptsLength = 6, .disableVelocity = true,
				.objectUpVectorVariance = -1, .cfg = survive_optimizer_precise_config()};
			SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(opt, 0);
			SurvivePose truth = {.Pos = {.1, -.2, .3}}, seed = truth;
			LinmathVec3d rotation = {.02, -.01, .03}, near_identity = {1e-9, -2e-9, 3e-9};
			quatfromaxisanglemag(truth.Rot, rotation);
			quatfromaxisanglemag(seed.Rot, near_identity);
			seed.Pos[0] += .01;
			if (sign) scalend(seed.Rot, seed.Rot, -1, 4);
			survive_optimizer_setup_pose(&opt, &seed, false, 1);
			SurvivePose cameras[3] = {{.Pos = {0, 0, 3}, .Rot = {1}},
				{.Pos = {-1, -.5, 2}, .Rot = {1}}, {.Pos = {1, .5, 2.5}, .Rot = {1}}};
			for (int lh = 0; lh < 3; lh++) survive_optimizer_setup_camera(&opt, lh, &cameras[lh], true, 1);
			survive_optimizer_parameter *pts = survive_optimizer_emplace_params(&opt, survive_optimizer_parameter_obj_points, 6);
			memcpy(pts->p, points, sizeof(points));
			survive_optimizer_parameter *cal = survive_optimizer_emplace_params(&opt, survive_optimizer_parameter_camera_parameters, 3);
			memset(cal->p, 0, cal->size * sizeof(FLT));
			BaseStationCal bcal[2] = {0};
			for (int lh = 0; lh < 3; lh++) for (int sensor = 0; sensor < 6; sensor++) {
				SurvivePose inverse = InvertPoseRtn(&cameras[lh]);
				LinmathPoint3d world, in_lh;
				ApplyPoseToPoint(world, &truth, points + sensor*3);
				ApplyPoseToPoint(in_lh, &inverse, world);
				FLT angles[2];
				opt.reprojectModel->reprojectXY(bcal, in_lh, angles);
				for (int axis = 0; axis < 2; axis++) {
					survive_optimizer_measurement *meas = survive_optimizer_emplace_meas(&opt, survive_optimizer_measurement_type_light);
					meas->light.lh = lh;
					meas->light.sensor_idx = sensor;
					meas->light.axis = axis;
					meas->light.value = angles[axis];
					meas->variance = .0001;
				}
			}
			CN_CREATE_STACK_MAT(R, 7, 7);
			mp_result result = {0};
			ASSERT_GT(survive_optimizer_run(&opt, &result, &R), 0);
			SurvivePose *solved = survive_optimizer_get_pose(&opt);
			ASSERT_EQ(survive_optimizer_has_unreliable_pose_covariance(&opt, &R), false);
			fprintf(stderr, "Near-identity seed gen%d sign%d: position error %.17g, norm %.17g, iterations %d\n",
				gen2, sign, dist3d(solved->Pos, truth.Pos), result.bestnorm, result.niter);
			ASSERT_EQ((dist3d(solved->Pos, truth.Pos) < 1e-8), true);
			// Compare physical rotations and transport the p/q covariance
			// cross terms when the output quaternion uses the other hemisphere.
			SurvivePose aligned = *solved;
			FLT hemisphere = quatinnerproduct(solved->Rot, truth.Rot) < 0 ? -1 : 1;
			scalend(aligned.Rot, aligned.Rot, hemisphere, 4);
			ASSERT_QUAT_EQ(aligned.Rot, truth.Rot);
			FLT aligned_covariance[49];
			for (int row = 0; row < 7; row++) for (int col = 0; col < 7; col++)
				aligned_covariance[row*7+col] = cnMatrixGet(&R, row, col) *
					((row < 3) != (col < 3) ? hemisphere : 1);
			if (sign == 0) {
				reference = aligned;
				memcpy(reference_covariance, aligned_covariance, sizeof(reference_covariance));
			} else {
				ASSERT_DOUBLE_ARRAY_EQ(3, reference.Pos, aligned.Pos);
				ASSERT_QUAT_EQ(reference.Rot, aligned.Rot);
				ASSERT_DOUBLE_ARRAY_EQ(49, reference_covariance, aligned_covariance);
			}
		}
	}
	return 0;
}

TEST(Optimizer, SearchLimitsApplyOnlyToFreePoseCoordinates) {
	FLT targets[] = {5, -6, 1.999, .25, 5, .25};
	for (int trial = 0; trial < 6; trial++) {
		survive_optimizer_settings local_settings = {.optimize_scale_threshold = -1, .use_quat_model = true,
			.disable_filter = true};
		survive_optimizer opt = {.settings = &local_settings, .reprojectModel = &survive_reproject_gen1_model,
			.poseLength = 2, .cameraLength = 1, .disableVelocity = true, .objectUpVectorVariance = -1,
			.cfg = survive_optimizer_precise_config()};
		SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(opt, 0);
		SurvivePose fixed = {.Pos = {20, 0, 0}, .Rot = {1}}, moving = {.Rot = {1}};
		survive_optimizer_setup_pose_n(&opt, &fixed, 0, true, 1);
		survive_optimizer_setup_pose_n(&opt, &moving, 1, true, 1);
		survive_optimizer_setup_camera(&opt, 0, &fixed, true, 1);
		// Constrain only the second object's x coordinate, using limits that
		// differ from the ordinary +/-20m. Fixed poses/cameras at their own
		// bounds must not invalidate this object's interior solution.
		opt.mp_parameters_info[7].fixed = false;
		opt.mp_parameters_info[7].limits[0] = -3;
		opt.mp_parameters_info[7].limits[1] = 2;
		if (trial == 4)
			opt.mp_parameters_info[7].limited[1] = false;
		for (int i = 0; i < 4; i++) {
			survive_optimizer_measurement *meas =
				survive_optimizer_emplace_meas(&opt, survive_optimizer_measurement_type_parameters_bias);
			meas->parameter_bias.parameter_index = 7;
			meas->parameter_bias.expected_value = targets[trial];
			meas->variance = .1;
		}
		if (trial == 5) {
			opt.mp_parameters_info[14].fixed = false;
			for (int i = 0; i < 4; i++) {
				survive_optimizer_measurement *meas =
					survive_optimizer_emplace_meas(&opt, survive_optimizer_measurement_type_parameters_bias);
				meas->parameter_bias.parameter_index = 14;
				meas->parameter_bias.expected_value = 25;
				meas->variance = .1;
			}
		}
		CN_CREATE_STACK_MAT(R, 21, 21);
		mp_result result = {0};
		ASSERT_EQ((survive_optimizer_run(&opt, &result, &R) > 0), true);
		ASSERT_EQ(survive_optimizer_has_pose_at_search_limit(&opt), trial < 2 || trial == 5);
		ASSERT_EQ(result.nfree, trial == 5 ? 2 : 1);
		if (trial < 2) {
			ASSERT_DOUBLE_EQ(opt.parameters[7], trial == 0 ? 2. : -3.);
			ASSERT_GT(result.bestnorm, 0.);
			// A successful constrained solve returns zero covariance even
			// though its observations disagree with the constrained position.
			ASSERT_DOUBLE_EQ(cnMatrixGet(&R, 0, 0), 0.);
		} else {
			ASSERT_DOUBLE_EQ(opt.parameters[7], targets[trial]);
			ASSERT_GT(cnMatrixGet(&R, 0, 0), 0.);
		}
	}
	return 0;
}

TEST(Optimizer, SingularPoseCovarianceDoesNotDescribeACompleteObservation) {
	for (int camera = 0; camera < 2; camera++) for (int complete = 0; complete < 2; complete++) {
		survive_optimizer_settings local_settings = {.optimize_scale_threshold = -1, .disable_filter = true};
		survive_optimizer opt = {.settings = &local_settings, .reprojectModel = &survive_reproject_gen1_model,
			.poseLength = 2, .cameraLength = 1, .disableVelocity = true, .objectUpVectorVariance = -1,
			.cfg = survive_optimizer_precise_config()};
		SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(opt, 0);
		SurvivePose anchor = {.Pos = {20, 0, 0}, .Rot = {1}}, seed = {.Pos = {.1, -.2, .3}};
		LinmathVec3d rotation = {.4, -.2, .3};
		quatfromaxisanglemag(seed.Rot, rotation);
		survive_optimizer_setup_pose_n(&opt, &anchor, 0, true, 1);
		survive_optimizer_setup_pose_n(&opt, &seed, 1, camera, 1);
		survive_optimizer_setup_camera(&opt, 0, &seed, !camera, 1);
		int start = camera ? survive_optimizer_get_camera_index(&opt) : 7;
		SurvivePose *selected = camera ? survive_optimizer_get_camera(&opt) : survive_optimizer_get_pose(&opt)+1;
		LinmathVec3d expected_rotation;
		quattoaxisanglemag(expected_rotation, selected->Rot);
		for (int coordinate = 0; coordinate < (complete ? 6 : 5); coordinate++) for (int repeat = 0; repeat < 4; repeat++) {
			survive_optimizer_measurement *meas =
				survive_optimizer_emplace_meas(&opt, survive_optimizer_measurement_type_parameters_bias);
			meas->parameter_bias.parameter_index = start+coordinate;
			meas->parameter_bias.expected_value = coordinate < 3 ? selected->Pos[coordinate] : expected_rotation[coordinate-3];
			meas->variance = .1;
		}
		CN_CREATE_STACK_MAT(R, 21, 21);
		mp_result result = {0};
		ASSERT_EQ((survive_optimizer_run(&opt, &result, &R) > 0), true);
		ASSERT_EQ(result.nfree, 6);
		ASSERT_EQ(R.rows, 7);
		// The unobserved axis-angle direction is mixed across quaternion
		// components: every returned diagonal remains strictly positive.
		for (int i = 0; i < R.rows; i++) ASSERT_GT(cnMatrixGet(&R, i, i), 0.);
		ASSERT_EQ(survive_optimizer_has_unreliable_pose_covariance(&opt, &R), !complete);
		ASSERT_EQ(survive_optimizer_has_unreliable_pose_covariance(&opt, NULL), false);
	}
	return 0;
}

TEST(Optimizer, OpticalHuberLossLeavesSmallResidualsAndPriorsUnchanged) {
	for (int gen2 = 0; gen2 < 2; gen2++) {
		for (int separate = 0; separate < 2; separate++) {
			survive_optimizer_settings config = {.use_quat_model = true, .disable_filter = true,
				.optimize_scale_threshold = -1, .disallow_pair_calc = separate};
			survive_optimizer opt = {.settings = &config, .poseLength = 1, .cameraLength = 1,
				.ptsLength = 1, .disableVelocity = true, .objectUpVectorVariance = -1, .light_huber = .0005,
				.reprojectModel = gen2 ? &survive_reproject_gen2_model : &survive_reproject_gen1_model};
			SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(opt, 0);
			SurvivePose pose = {.Rot = {1}}, lh = {.Pos = {0, 0, 4}, .Rot = {1}};
			survive_optimizer_setup_pose(&opt, &pose, true, 1);
			survive_optimizer_setup_camera(&opt, 0, &lh, true, 1);
			FLT point[3] = {.02, .07, .01}, angles[2], errors[2] = {.0001, -.003};
			SurvivePose inverse = InvertPoseRtn(&lh);
			LinmathPoint3d in_lh;
			ApplyPoseToPoint(in_lh, &inverse, point);
			BaseStationCal bcal[2] = {0};
			opt.reprojectModel->reprojectXY(bcal, in_lh, angles);
			survive_optimizer_parameter *pts = survive_optimizer_emplace_params(&opt, survive_optimizer_parameter_obj_points, 1);
			memcpy(pts->p, point, sizeof(point));
			survive_optimizer_parameter *cal = survive_optimizer_emplace_params(&opt, survive_optimizer_parameter_camera_parameters, 1);
			memset(cal->p, 0, cal->size * sizeof(FLT));
			// Ordinary priors are not part of the robust optical loss.
			survive_optimizer_measurement *prior = survive_optimizer_emplace_meas(&opt, survive_optimizer_measurement_type_parameters_bias);
			prior->parameter_bias.parameter_index = 0;
			prior->parameter_bias.expected_value = 1;
			prior->variance = .1;
			for (int axis = 0; axis < 2; axis++) {
				survive_optimizer_measurement *meas = survive_optimizer_emplace_meas(&opt, survive_optimizer_measurement_type_light);
				meas->light.axis = axis;
				meas->light.value = angles[axis]-errors[axis];
				meas->variance = .0001;
			}
			ASSERT_DOUBLE_EQ(survive_optimizer_current_norm(&opt), 100 + 1 + 275.);
			opt.light_huber = 0;
			ASSERT_DOUBLE_EQ(survive_optimizer_current_norm(&opt), 100 + 1 + 900.);
			// A coherent initial discrepancy widens the transition. Its scale
			// remains fixed when the objective is evaluated again.
			opt.light_huber = -.0005;
			ASSERT_DOUBLE_EQ(survive_optimizer_current_norm(&opt), 100 + 1 + 900.);
			ASSERT_DOUBLE_EQ(opt.light_huber, 6 * (.0001 + .003) / 2);
			FLT transition = opt.light_huber;
			ASSERT_DOUBLE_EQ(survive_optimizer_current_norm(&opt), 100 + 1 + 900.);
			ASSERT_DOUBLE_EQ(opt.light_huber, transition);
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

TEST(Optimizer, OpticalHuberRecoversPoseWithContaminatedBeam) {
	for (int gen2 = 0; gen2 < 2; gen2++) {
		for (int quaternion = 0; quaternion < 2; quaternion++) {
			SurviveContext ctx = {0};
			SurviveObject so = {.ctx = &ctx};
			FLT errors[6] = {0};
			int retained[6] = {0};
			for (int mode = 0; mode < 6; mode++) {
				survive_optimizer_settings config = {.use_quat_model = quaternion, .disable_filter = mode < 3,
					.optimize_scale_threshold = -1};
				survive_optimizer opt = {.settings = &config, .poseLength = 1, .cameraLength = 3,
					.ptsLength = 6, .disableVelocity = true, .objectUpVectorVariance = -1,
					.light_huber = mode == 5 ? -.0003 : (mode == 1 || mode == 2 ? .0003 : 0),
					.light_outlier_ratio = mode == 3 || mode == 4 ? 6 : 0,
					.light_outlier_floor = .01, .cfg = survive_optimizer_precise_config(),
					.reprojectModel = gen2 ? &survive_reproject_gen2_model : &survive_reproject_gen1_model};
				SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(opt, 0);
				opt.sos[0] = &so;
				SurvivePose truth = {.Pos = {.1, -.2, .3}}, start;
				FLT axis[] = {.2, -.1, .3};
				quatfromaxisanglemag(truth.Rot, axis);
				start = truth;
				start.Pos[0] += mode >= 4 ? .3 : .01;
				survive_optimizer_setup_pose(&opt, &start, false, mode == 2 ? 0 : 1);
				SurvivePose cameras[3] = {{.Pos = {0, 0, 3}, .Rot = {1}},
					{.Pos = {-1, -.5, 2}, .Rot = {1}}, {.Pos = {1, .5, 2.5}, .Rot = {1}}};
				for (int lh = 0; lh < 3; lh++)survive_optimizer_setup_camera(&opt, lh, &cameras[lh], true, 1);
				survive_optimizer_parameter *pts = survive_optimizer_emplace_params(&opt, survive_optimizer_parameter_obj_points, 6);
				memcpy(pts->p, points, sizeof(points));
				survive_optimizer_parameter *cal = survive_optimizer_emplace_params(&opt, survive_optimizer_parameter_camera_parameters, 3);
				memset(cal->p, 0, cal->size * sizeof(FLT));
				BaseStationCal bcal[2] = {0};
				for (int lh = 0; lh < 3; lh++)for (int sensor = 0; sensor < 6; sensor++) {
					SurvivePose inverse = InvertPoseRtn(&cameras[lh]);
					LinmathPoint3d world, in_lh;
					ApplyPoseToPoint(world, &truth, points + sensor*3);
					ApplyPoseToPoint(in_lh, &inverse, world);
					FLT angles[2];
					opt.reprojectModel->reprojectXY(bcal, in_lh, angles);
					for (int axis = 0; axis < 2; axis++) {
						survive_optimizer_measurement *meas = survive_optimizer_emplace_meas(&opt, survive_optimizer_measurement_type_light);
						meas->light.lh = lh;
						meas->light.sensor_idx = sensor;
						meas->light.axis = axis;
						meas->light.value = angles[axis]+(mode < 4 && lh == 0 && sensor == 0 && axis == 1 ? .08 : 0);
						meas->variance = .0001;
					}
				}
				mp_result result = {0};
				ASSERT_GT(survive_optimizer_run(&opt, &result, 0), 0);
				retained[mode] = opt.measurementsCnt;
				errors[mode] = dist3d(survive_optimizer_get_pose(&opt)->Pos, truth.Pos);
			}
			ASSERT_GT(errors[0], errors[1]*10);
			ASSERT_GT(.001, errors[1]);
			ASSERT_GT(.001, errors[2]);
			if (!quaternion) ASSERT_GT(.00002, fabs(errors[1]-errors[2]));
			ASSERT_GT(.00001, errors[3]);
			ASSERT_EQ(retained[3], 35);
			// A coherent large initial-pose disagreement must keep every beam.
			ASSERT_GT(.0001, errors[4]);
			ASSERT_EQ(retained[4], 36);
			ASSERT_GT(.0001, errors[5]);
			ASSERT_EQ(retained[5], 36);
		}
	}
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
