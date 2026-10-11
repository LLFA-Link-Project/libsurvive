// Exercise acquisition and propagation directly, without USB or device JSON.
#include "../disambiguator_statebased.c"
#include "test_case.h"

static void init_state(Disambiguator_data_t *d, SurviveObject *so, SurviveContext *ctx,
					   Global_Disambiguator_data_t *g) {
	memset(d, 0, sizeof(*d));
	memset(so, 0, sizeof(*so));
	memset(ctx, 0, sizeof(*ctx));
	memset(g, 0, sizeof(*g));
	g->verbosity = 1000;
	g->ctx = ctx;
	ctx->disambiguator_data = g;
	so->ctx = ctx;
	d->so = so;
}

static void add_cycle(Disambiguator_data_t *d, uint32_t base, bool wide_sweeps, int station) {
	for (int s = LS_UNKNOWN + 1; s < LS_END; s++) {
		if (LS_Params[s].is_sweep) {
			if (wide_sweeps && s == LS_SweepAX) {
				// Four separate long sweep candidates in the 12-candidate
				// failure pattern, at different sensor crossing times.
				for (int i = 0; i < 4; i++)
					AddSyncHistory(d, (LightcapElement){.timestamp = base + 270000 + i * 3500,
														  .length = 5000 + i * 400}, false);
			}
		} else if (station < 0 || LS_Params[s].lh == station) {
			AddSyncHistory(d, (LightcapElement){.timestamp = base + LSParam_offset_for_state(s),
												  .length = ACODE_TIMING(LSParam_acode(s))}, true);
		}
	}
}

TEST(Disambiguator, WideSweepsPreserveTwelveSyncs) {
	SurviveContext ctx;
	SurviveObject so;
	Global_Disambiguator_data_t g;
	Disambiguator_data_t d;
	init_state(&d, &so, &ctx, &g);
	uint32_t base = 10000000, mod = 0;
	bool single = false;
	add_cycle(&d, base, true, -1);
	// Eight syncs plus four wide sweeps must not lower the evidence threshold.
	ASSERT_EQ(find_relative_offset(&d, &mod, &single), LS_UNKNOWN);
	add_cycle(&d, base + 1600000, true, -1);
	ASSERT_EQ(find_inliers(&d, base, false), 16);
	ASSERT_EQ(find_relative_offset(&d, &mod, &single), LS_WaitLHB_ACode5);
	ASSERT_EQ(single, false);
	ASSERT_EQ(apply_mod_offset(base, mod, LS_END), 0);
	return 0;
}

TEST(Disambiguator, OccludedStationAndCleanInput) {
	for (int station = -1; station < 2; station++) {
		SurviveContext ctx;
		SurviveObject so;
		Global_Disambiguator_data_t g;
		Disambiguator_data_t d;
		init_state(&d, &so, &ctx, &g);
		uint32_t base = 10000000, mod = 0;
		bool single = true;
		for (int cycle = 0; cycle < 3; cycle++)
			add_cycle(&d, base + cycle * 1600000, false, station);
		ASSERT_EQ((find_relative_offset(&d, &mod, &single) != LS_UNKNOWN), true);
		ASSERT_EQ(single, false);
		// With just one visible station its physical index is ambiguous:
		// the equivalent model swaps stations and shifts by 820000 ticks.
		uint32_t phase = apply_mod_offset(base, mod, LS_END);
		ASSERT_EQ((phase == 0 || (station >= 0 && (phase == 780000 || phase == 820000))), true);
	}
	return 0;
}

TEST(Disambiguator, SingleStation60Hz) {
	SurviveContext ctx;
	SurviveObject so;
	Global_Disambiguator_data_t g;
	Disambiguator_data_t d;
	init_state(&d, &so, &ctx, &g);
	uint32_t base = 10000000, mod = 0;
	bool single = false;
	for (int cycle = 0; cycle < 6; cycle++) {
		AddSyncHistory(&d, (LightcapElement){.timestamp = base + cycle * 800000 + 20000, .length = 2750}, true);
		AddSyncHistory(&d, (LightcapElement){.timestamp = base + cycle * 800000 + 420000, .length = 3250}, true);
	}
	ASSERT_EQ(find_relative_offset(&d, &mod, &single), LS_WaitLHA_ACode1);
	ASSERT_EQ(single, true);
	ASSERT_EQ(apply_mod_offset(base, mod, LS_WaitLHB_ACode0), 0);
	return 0;
}

TEST(Disambiguator, CleanAcquisitionDoesNotWaitForOldNoise) {
	SurviveContext ctx;
	SurviveObject so;
	Global_Disambiguator_data_t g;
	Disambiguator_data_t d;
	init_state(&d, &so, &ctx, &g);
	uint32_t base = 10000000, mod = 0;
	bool single = true;
	for (int i = 0; i < 12; i++)
		AddSyncHistory(&d, (LightcapElement){.timestamp = base - 100000 + i * 5000, .length = 2750}, true);
	int count = 0;
	for (int cycle = 0; count < 12; cycle++) {
		for (int s = 1; s < LS_END && count < 12; s++) {
			if (LS_Params[s].is_sweep)
				continue;
			AddSyncHistory(&d, (LightcapElement){.timestamp = base + cycle * 1600000 + LSParam_offset_for_state(s),
												  .length = ACODE_TIMING(LSParam_acode(s))}, true);
			count++;
			if (count < 12)
				ASSERT_EQ(find_relative_offset(&d, &mod, &single), LS_UNKNOWN);
		}
	}
	ASSERT_EQ(find_relative_offset(&d, &mod, &single), LS_WaitLHA_ACode1);
	ASSERT_EQ(single, false);
	ASSERT_EQ(apply_mod_offset(base, mod, LS_END), 0);
	return 0;
}

TEST(Disambiguator, RejectConflictingSyncAndSinglePhaseNoise) {
	SurviveContext ctx;
	SurviveObject so;
	Global_Disambiguator_data_t g;
	Disambiguator_data_t d;
	init_state(&d, &so, &ctx, &g);
	uint32_t base = 10000000, mod = 0;
	bool single = false;
	for (int cycle = 0; cycle < 2; cycle++)
		add_cycle(&d, base + cycle * 1600000, true, -1);
	// Both station identities are determined when both broadcast sequences
	// are visible. A phase shift may not hide one sequence in sweep windows.
	ASSERT_EQ(find_inliers(&d, base + 820000, false), 0);
	ASSERT_EQ(find_inliers(&d, base + 780000, false), 0);
	ASSERT_EQ(find_inliers(&d, base + 20000, false), 0);
	d.sync_history[(d.sync_offset + SYNC_HISTORY_LEN - 1) % SYNC_HISTORY_LEN].length = 3250; // Wrong skip bit at the newest sync phase.
	ASSERT_EQ(find_relative_offset(&d, &mod, &single), LS_UNKNOWN);
	init_state(&d, &so, &ctx, &g);
	for (int i = 0; i < SYNC_HISTORY_LEN; i++)
		AddSyncHistory(&d, (LightcapElement){.timestamp = base + i * 1600000, .length = 2750}, true);
	ASSERT_EQ(find_relative_offset(&d, &mod, &single), LS_UNKNOWN);
	return 0;
}

TEST(Disambiguator, BroadcastGroupingAcrossRollover) {
	SurviveContext ctx;
	SurviveObject so;
	Global_Disambiguator_data_t g;
	Disambiguator_data_t d;
	init_state(&d, &so, &ctx, &g);
	LightcapElement a = {.sensor_id = 0, .timestamp = UINT32_MAX - 50, .length = 4000};
	LightcapElement b = {.sensor_id = 1, .timestamp = 5, .length = 3990};
	ASSERT_EQ(overlap_area(&a, &b), 3944);
	ASSERT_EQ(overlaps(&a, &b), true);
	RegisterSync(&d, &b);
	RegisterSync(&d, &a); // Slightly out-of-order sensor delivery.
	ASSERT_EQ(d.first_sync_timestamp, a.timestamp);
	ASSERT_EQ(d.latest_sync_timestamp, b.timestamp);
	ASSERT_EQ(sync_is_broadcast(&d), true);
	ResetSync(&d);
	RegisterSync(&d, &a);
	RegisterSync(&d, &a); // Duplicate pulses are not multiple sensors.
	ASSERT_EQ(sync_is_broadcast(&d), false);
	b.timestamp = a.timestamp + 437;
	RegisterSync(&d, &b);
	ASSERT_EQ(sync_is_broadcast(&d), false);
	b.timestamp = a.timestamp + 0x80000000u;
	ASSERT_EQ(overlap_area(&a, &b), 0);
	return 0;
}

TEST(Disambiguator, ClockRolloverAndOlderCandidates) {
	for (int end = LS_WaitLHB_ACode0; end <= LS_END; end += LS_END - LS_WaitLHB_ACode0) {
		int period = LSParam_offset_for_state(end);
		ASSERT_EQ(apply_mod_offset(0, UINT32_MAX, end), 1);
		ASSERT_EQ(apply_mod_offset(UINT32_MAX, 0, end), period - 1);
		ASSERT_EQ(apply_mod_offset(17, 17, end), 0);
		ASSERT_EQ(apply_mod_offset(0xfffffff0, 32, end), period - 48);
	}
	SurviveContext ctx;
	SurviveObject so;
	Global_Disambiguator_data_t g;
	Disambiguator_data_t d;
	init_state(&d, &so, &ctx, &g);
	uint32_t base = UINT32_MAX - 2000000, mod = 0;
	bool single = true;
	add_cycle(&d, base, true, -1);
	add_cycle(&d, base + 1600000, true, -1);
	ASSERT_EQ(find_inliers(&d, base, false), 16);
	ASSERT_EQ((find_relative_offset(&d, &mod, &single) != LS_UNKNOWN), true);
	ASSERT_EQ(single, false);
	ASSERT_EQ(apply_mod_offset(base, mod, LS_END), 0);
	return 0;
}

static uint32_t stream_base;
static int stream_hits[2][2], stream_errors;

static void check_light(SurviveObject *so, int sensor, int acode, int timeinsweep,
						uint32_t timestamp, uint32_t length, uint32_t lh) {
	if (sensor < 0)
		return;
	uint32_t phase = (timestamp - stream_base) % 1600000;
	int quarter = phase / 400000;
	int expected_lh = quarter / 2, axis = quarter & 1;
	int sweep_state = LS_SweepAX + quarter * 3;
	int expected_time = phase + length / 2 - LSParam_offset_for_state(sweep_state - 1 - expected_lh);
	if (lh != expected_lh || acode != LSParam_acode(sweep_state) || timeinsweep != expected_time)
		stream_errors++;
	else
		stream_hits[lh][axis]++;
}

TEST(Disambiguator, WideSweepStreamKeepsStationAxisAndTime) {
	for (int rollover = 0; rollover < 2; rollover++) {
		SurviveContext ctx;
		SurviveObject so;
		Global_Disambiguator_data_t g;
		Disambiguator_data_t *d = calloc(1, sizeof(*d) + 28 * sizeof(LightcapElement));
		init_state(d, &so, &ctx, &g);
		ctx.lightproc = check_light;
		so.sensor_ct = 28;
		so.timebase_hz = 48000000;
		so.disambiguator_data = d;
		d->stabalize = 200;
		stream_base = rollover ? UINT32_MAX - 3000000 : 10000000;
		stream_errors = 0;
		memset(stream_hits, 0, sizeof(stream_hits));
		for (int cycle = 0; cycle < 24; cycle++) {
			uint32_t base = stream_base + cycle * 1600000;
			for (int s = 1; s < LS_END; s++) {
				for (int sensor = 0; sensor < 28; sensor++) {
					LightcapElement le = {.sensor_id = sensor,
						.timestamp = base + LSParam_offset_for_state(s) +
							(LS_Params[s].is_sweep ? 80000 + sensor * 1000 : sensor * 3),
						.length = LS_Params[s].is_sweep ? (s == LS_SweepAX ? 6000 : 400)
							: ACODE_TIMING(LSParam_acode(s)) + (cycle & 1) * 1000};
					DisambiguatorStateBased(&so, &le);
				}
			}
		}
		ASSERT_EQ(stream_errors, 0);
		ASSERT_EQ(d->stats.confidence_resets, 0);
		ASSERT_EQ(d->stats.sync_time_error, 0);
		for (int lh = 0; lh < 2; lh++)
			for (int axis = 0; axis < 2; axis++)
				ASSERT_EQ((stream_hits[lh][axis] > 0), true);
		free(d);
	}
	return 0;
}
