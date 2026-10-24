#include "../survive_kalman_tracker.h"
#include "test_case.h"

static size_t pose_count;
static survive_long_timecode callback_tick;
static void receive_pose(SurviveObject *so, survive_long_timecode tick, const SurvivePose *pose) {
	pose_count++;
	callback_tick = so->OutPose_timecode = tick;
	so->OutPoseIMU = *pose;
}

// A stationary, valid filter state exercises the real report path, including
// non-IMU callers, without hardware, optimization, or a wall clock.
static int report_sequence(survive_long_timecode start, uint64_t period, FLT minimum,
                           size_t expected, bool interleave_light) {
	SurviveContext ctx = {.imuposeproc = receive_pose};
	SurviveObject so = {.ctx = &ctx, .timebase_hz = 48000000, .imu_freq = 250, .conf = "{}"};
	SurviveKalmanTracker tracker = {.so = &so, .min_report_time = minimum,
	                              .state = {.Pose = {.Rot = {1}}, .IMUBias = {.IMUCorrection = {1}}}};
	FLT covariance[13 * 13] = {0};
	for (int i = 0; i < 13; i++)
		covariance[i * 13 + i] = 1e-6;
	tracker.model.state_cnt = tracker.model.error_state_size = 13;
	tracker.model.state = cnVec(13, (FLT *)&tracker.state);
	tracker.model.P = cnMat(13, 13, covariance);
	pose_count = 0;
	callback_tick = 0;
	for (int i = 1; i <= 1000; i++) {
		PoserData pd = {.pt = POSERDATA_IMU, .timecode = start + i * period};
		tracker.model.t = pd.timecode / (FLT)so.timebase_hz;
		survive_kalman_tracker_report_state(&pd, &tracker);
		if (interleave_light) {
			// Same-tick optical corrections, out-of-order events, and updates
			// well inside a report period must not multiply callbacks.
			pd.pt = POSERDATA_LIGHT_GEN2;
			survive_kalman_tracker_report_state(&pd, &tracker);
			pd.timecode--;
			survive_kalman_tracker_report_state(&pd, &tracker);
			pd.timecode += period / 4 + 1;
			tracker.model.t = pd.timecode / (FLT)so.timebase_hz;
			survive_kalman_tracker_report_state(&pd, &tracker);
		}
	}
	ASSERT_EQ(pose_count, expected);
	ASSERT_EQ(tracker.stats.reported_poses, expected);
	ASSERT_EQ(callback_tick <= start + 1000 * period, true);
	ASSERT_DOUBLE_EQ(tracker.min_report_time, minimum);
	for (int i = 0; i < 13; i++)
		ASSERT_EQ(isfinite(((FLT *)&tracker.reported_state_variance)[i]), true);
	ASSERT_DOUBLE_EQ(so.OutPoseIMU.Rot[0], 1.);
	return 0;
}

TEST(KalmanReporting, NominalCadenceDoesNotHalveFastIMU) {
	const uint64_t periods[] = {191488, 192000, 192256}; // 3.989333 / 4.000 / 4.005333 ms
	const uint64_t starts[] = {0, (UINT64_C(1) << 32) - 1000000, UINT64_C(48000000) * 86400,
	                           UINT64_C(1) << 58};
	for (size_t i = 0; i < sizeof(periods) / sizeof(*periods); i++) {
		for (size_t j = 0; j < sizeof(starts) / sizeof(*starts); j++) {
			int result = report_sequence(starts[j], periods[i], -1, 1000, false);
			if (result) return result;
		}
	}
	return 0;
}

TEST(KalmanReporting, ExplicitMinimumStillLimitsOutput) {
	int result = report_sequence(UINT64_C(48000000) * 86400, 191488, .004, 500, false);
	if (result) return result;
	result = report_sequence(UINT64_C(48000000) * 86400, 192000, .004, 1000, false);
	if (result) return result;
	return report_sequence(UINT64_C(48000000) * 86400, 192256, .008, 500, false);
}

TEST(KalmanReporting, OpticalUpdatesDoNotDuplicateReports) {
	return report_sequence(UINT64_C(48000000) * 86400, 191488, -1, 1000, true);
}

TEST(KalmanReporting, ZeroMinimumHandlesSubUlpTickDifferences) {
	return report_sequence(UINT64_C(1) << 58, 1, 0, 1000, false);
}

TEST(KalmanReporting, DefaultStillThrottlesFasterUpdates) {
	return report_sequence(UINT64_C(48000000) * 86400, 96000, -1, 500, true);
}
