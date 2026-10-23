#include "survive_reproject.h"
#include "test_case.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../driver_vive.h"
#include "../survive_internal.h"
#include "../survive_private.h"

static int imu_reports;
static FLT imu_sample[6];
static int sync_reports;
static void capture_sync(SurviveObject *so, survive_channel channel, survive_timecode timecode, bool ootx, bool gen) {
	sync_reports++;
}
static void capture_imu(SurviveObject *so, int mask, const FLT *sample, survive_timecode timecode, int id) {
	imu_reports++;
	memcpy(imu_sample, sample, sizeof(imu_sample));
	so->activations.last_imu = timecode;
}

TEST(ViveDriver, PassiveWatchmanV2) {
	SurviveContext context = {0};
	SurviveContext *ctx = &context;
	struct SurviveContext_private private_context = {.poll_sema = OGCreateSema()};
	ctx->private_members = &private_context;
	OGUnlockSema(private_context.poll_sema);
#define SURVIVE_HOOK_PROCESS_DEF(hook) survive_install_##hook##_fn(ctx, 0);
#include "survive_hooks.h"
	ctx->log_target = stderr;
	survive_install_raw_imu_fn(ctx, capture_imu);
	SurviveObject so = {.ctx = ctx, .conf = "{}", .object_subtype = SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R};
	survive_vive_register_driver(&so, 0x28de, 0x2101);

	// One RF subreport: MSB, size, LSB, flags, then a 13-byte IMU sample.
	uint8_t report[59] = {0x23, 0x01, 15, 0x02, 0x80, 0x03, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0};
	SurviveUSBInterface si = {.ctx = ctx, .assoc_obj = &so, .buffer = report,
							 .actual_len = 30, .which_interface_am_i = USB_IF_WATCHMAN1};
	imu_reports = 0;
	survive_data_cb(1000000, &si);
	ASSERT_EQ(imu_reports, 1);
	for (int i = 0; i < 6; i++) {
		ASSERT_EQ(imu_sample[i], i + 1);
	}

	// A complete USB packet with an incomplete embedded IMU must not emit a sample.
	report[2] = 14;
	survive_data_cb(1001000, &si);
	ASSERT_EQ(imu_reports, 1);
	report[2] = 0;
	survive_data_cb(1002000, &si);
	ASSERT_EQ(imu_reports, 1);
	report[2] = 28;
	survive_data_cb(1003000, &si);
	ASSERT_EQ(imu_reports, 1);
	report[2] = 15;
	si.actual_len = 17;
	survive_data_cb(1004000, &si);
	ASSERT_EQ(imu_reports, 1);

	// Both halves of a double report must be present before either is decoded.
	report[0] = 0x24;
	memcpy(report + 30, report + 1, 29);
	si.actual_len = 46;
	survive_data_cb(1005000, &si);
	ASSERT_EQ(imu_reports, 1);
	si.actual_len = 59;
	survive_data_cb(1006000, &si);
	ASSERT_EQ(imu_reports, 3);

	// Settle an actual mode switch before testing unrelated control transfers.
	survive_install_sync_fn(ctx, capture_sync);
	uint8_t mode[8] = {4, 1};
	survive_data_on_setup_write(&so, 0x21, 9, 0x304, 0, mode, sizeof(mode));
	report[0] = 0x23;
	si.actual_len = 30;
	for (int i = 0; i < 40; i++)
		survive_data_cb(1007000 + i * 1000, &si);
	uint8_t optical[30] = {0x23, 1, 7, 2, 0x10, 0x01, 0x00, 0x04, 0x00, 0x00};
	si.buffer = optical;
	sync_reports = 0;
	survive_data_cb(1050000, &si);
	ASSERT_EQ(sync_reports, 1);
	// Reading a feature and writing haptics must not restart optical quarantine.
	survive_data_on_setup_write(&so, 0xa1, 1, 0x305, 0, 0, 0);
	survive_data_cb(1051000, &si);
	ASSERT_EQ(sync_reports, 2);
	uint8_t haptic[11] = {0xff, 0x8f, 8};
	survive_data_on_setup_write(&so, 0x21, 9, 0x3ff, 0, haptic, sizeof(haptic));
	survive_data_cb(1052000, &si);
	ASSERT_EQ(sync_reports, 3);
	free(so.driver);
	OGDeleteSema(private_context.poll_sema);
	return 0;
}

TEST(ViveDriver, TestWatchmanParsing) {

	{
		uint8_t readdata[] = {0xff, 0x09, 0x00, 0x04, 0x00, 0x38, 0xb8, 0xec, 0xe4, 0x9f};

		LightcapElement les[10];
		int cnt = parse_watchman_lightcap(0, "WW0", 0, 0, readdata, sizeof(readdata), les, 10);
	}

	{
		uint8_t readdata[] = {0x00, 0x6f, 0xfd, 0x83, 0xff};

		LightcapElement les[10];
		int cnt = parse_watchman_lightcap(0, "WW0", 224, 3761897504, readdata, sizeof(readdata), les, 10);
	}

	{
		uint8_t readdata[] = {0x88, 0x81, 0xa1, 0x10, 0x00, 0xd3, 0x06, 0x93, 0x03, 0xa3, 0x06, 0xf3, 0x06,
							  0x83, 0x01, 0xd6, 0x06, 0xe4, 0xa8, 0x0c, 0xd9, 0x07, 0xc1, 0x92, 0xd2};
		LightcapElement les[10];
		int cnt = parse_watchman_lightcap(0, "WW0", 224, 3761897504, readdata, sizeof(readdata), les, 10);
	}
	return 0;
}
