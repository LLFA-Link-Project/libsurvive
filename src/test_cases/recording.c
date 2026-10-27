#include "test_case.h"
#include "../survive_internal.h"
#include "../survive_private.h"
#include "../survive_config.h"
#include "../survive_recording.h"
#include "../survive_kalman_tracker.h"
#include "../survive_gz.h"
#include <ctype.h>
#include <unistd.h>

#define RECORD_EQ(a, b) do { \
	if ((a) != (b)) { \
		fprintf(stderr, "Recording check failed at line %d\n", __LINE__); \
		return -1; \
	} \
} while (0)

#define RECORDS_PER_WRITER 3000
#define WRITERS 4

struct writer_args {
	SurviveContext *ctx;
	int id;
};

static double recording_clock(const SurviveContext *ctx, void *user) { return 1.; }

static void *write_records(void *arg) {
	struct writer_args *a = arg;
	SurviveObject so = {.ctx = a->ctx};
	snprintf(so.codename, sizeof(so.codename), "T%d", a->id);
	for (int i = 0; i < RECORDS_PER_WRITER; i++) {
		if (i % 3 == 0) {
			FLT values[] = {a->id, i, 3, 4};
			CnMat matrix = cnMat(2, 2, values);
			survive_recording_write_matrix(a->ctx->recptr, &so, 0, "test", &matrix);
		} else if (i % 3 == 1) {
			char config[128];
			int len = snprintf(config, sizeof(config), "{\n\"writer\":%d,\"sequence\":%d}", a->id, i);
			survive_recording_config_process(&so, config, len);
		} else {
			survive_recording_write_to_output(a->ctx->recptr, "%s TEST %d %d\n", so.codename, a->id, i);
		}
	}
	return NULL;
}

static int concurrent_records(bool compressed) {
	char temporary_path[] = "/tmp/survive-recording-XXXXXX";
	int fd = mkstemp(temporary_path);
	RECORD_EQ(fd >= 0, true);
	close(fd);
	char path[128];
	snprintf(path, sizeof(path), "%s%s", temporary_path, compressed ? ".gz" : "");
	if (compressed)
		unlink(temporary_path);
	struct SurviveContext_private private = {.runTimeFn = recording_clock};
	config_group temporary = {.write_lock = OGCreateMutex()};
	SurviveContext ctx = {.private_members = &private, .temporary_config_values = &temporary};
	survive_configs(&ctx, "record", SC_SET | SC_OVERRIDE, path);
	survive_install_recording(&ctx);
	RECORD_EQ(ctx.recptr != NULL, true);
	struct writer_args args[WRITERS];
	og_thread_t threads[WRITERS];
	for (int i = 0; i < WRITERS; i++) {
		args[i] = (struct writer_args){&ctx, i};
		threads[i] = OGCreateThread(write_records, "recording test", &args[i]);
	}
	for (int i = 0; i < WRITERS; i++)
		OGJoinThread(threads[i]);
	survive_destroy_recording(&ctx);
	gzFile input = gzopen(path, "rb");
	RECORD_EQ(input != NULL, true);
	bool seen[WRITERS][RECORDS_PER_WRITER] = {0};
	char line[1024];
	int total = 0;
	while (gzgets(input, line, sizeof(line))) {
		double ts;
		char device[32], op[32];
		RECORD_EQ(sscanf(line, "%lf %31s %31s", &ts, device, op), 3);
		if (strcmp(device, "OPTION") == 0 || strcmp(device, "INFO") == 0)
			continue;
		int id = -1, sequence = -1, used = 0;
		if (strcmp(op, "TEST") == 0) {
			RECORD_EQ(sscanf(line, "%lf %31s TEST %d %d %n", &ts, device, &id, &sequence, &used), 4);
		} else if (strcmp(op, "CONFIG") == 0) {
			RECORD_EQ(sscanf(line, "%lf %31s CONFIG { \"writer\":%d,\"sequence\":%d} %n",
				&ts, device, &id, &sequence, &used), 4);
		} else {
			double writer, iteration, v3, v4;
			RECORD_EQ(strcmp(op, "DATA_MATRIX"), 0);
			RECORD_EQ(sscanf(line, "%lf %31s DATA_MATRIX test 2 2 %lf %lf %lf %lf %n",
				&ts, device, &writer, &iteration, &v3, &v4, &used), 6);
			id = writer;
			sequence = iteration;
			ASSERT_DOUBLE_EQ(v3, 3.);
			ASSERT_DOUBLE_EQ(v4, 4.);
		}
		RECORD_EQ(used > 0 && line[used] == 0, true);
		RECORD_EQ(id >= 0 && id < WRITERS && sequence >= 0 && sequence < RECORDS_PER_WRITER, true);
		RECORD_EQ(device[0] == 'T' && device[1] == '0' + id && device[2] == 0, true);
		RECORD_EQ(seen[id][sequence], false);
		seen[id][sequence] = true;
		total++;
	}
	RECORD_EQ(gzclose(input), 0);
	unlink(path);
	RECORD_EQ(total, WRITERS * RECORDS_PER_WRITER);
	for (int i = 0; i < temporary.used_entries; i++) {
		RECORD_EQ(temporary.config_entries[i].update_list, NULL);
		free(temporary.config_entries[i].tag);
		free(temporary.config_entries[i].data);
	}
	free(temporary.config_entries);
	OGDeleteMutex(temporary.write_lock);
	return 0;
}

TEST(Recording, ConcurrentPlainRecords) { return concurrent_records(false); }
TEST(Recording, ConcurrentGzipRecords) { return concurrent_records(true); }

TEST(Recording, MotionStateKeepsMeasurementEpochAndExportedAcceleration) {
	char path[] = "/tmp/survive-motion-recording-XXXXXX";
	int fd = mkstemp(path);
	RECORD_EQ(fd >= 0, true);
	close(fd);
	struct SurviveContext_private private = {.runTimeFn = recording_clock};
	config_group temporary = {.write_lock = OGCreateMutex()};
	SurviveContext ctx = {.private_members = &private, .temporary_config_values = &temporary};
	survive_configs(&ctx, "record", SC_SET | SC_OVERRIDE, path);
	survive_configi(&ctx, "record-tracking-state", SC_SET | SC_OVERRIDE, 1);
	survive_install_recording(&ctx);
	SurviveKalmanTracker tracker = {.state = {.Velocity = {.Pos = {.01, -.02, .03},
		.AxisAngleRot = {.1, -.2, .3}}}};
	SurviveObject so = {.ctx = &ctx, .timebase_hz = 48000000, .tracker = &tracker,
		.acceleration = {.4, -.5, .6}, .OutPoseIMU = {.Rot = {1}}};
	strcpy(so.codename, "WM0");
	uint64_t tick = (UINT64_C(1) << 58) + 192000;
	survive_recording_raw_pose_process(&so, tick, &so.OutPoseIMU);
	survive_destroy_recording(&ctx);
	FILE *input = fopen(path, "r");
	RECORD_EQ(input != NULL, true);
	char line[1024];
	bool found = false;
	while (fgets(line, sizeof(line), input)) {
		if (!strstr(line, " POSE_MOTION ")) continue;
		unsigned long long recorded_tick;
		int hz;
		double receive, values[9];
		RECORD_EQ(sscanf(line, "%lf WM0 POSE_MOTION %llu %d %lf %lf %lf %lf %lf %lf %lf %lf %lf",
			&receive, &recorded_tick, &hz, &values[0], &values[1], &values[2], &values[3], &values[4],
			&values[5], &values[6], &values[7], &values[8]), 12);
		RECORD_EQ(recorded_tick, tick);
		RECORD_EQ(hz, 48000000);
		for (int i = 0; i < 6; i++) ASSERT_DOUBLE_EQ(values[i], ((FLT *)&tracker.state.Velocity)[i]);
		for (int i = 0; i < 3; i++) ASSERT_DOUBLE_EQ(values[i + 6], so.acceleration[i]);
		found = true;
	}
	fclose(input);
	unlink(path);
	RECORD_EQ(found, true);
	for (int i = 0; i < temporary.used_entries; i++) {
		free(temporary.config_entries[i].tag);
		free(temporary.config_entries[i].data);
	}
	free(temporary.config_entries);
	OGDeleteMutex(temporary.write_lock);
	return 0;
}
