#include "test_case.h"
#include "../survive_config.h"

static int resets;
static void observe_reset(SurviveContext *ctx) { resets++; }
#define survive_reset_lighthouse_positions observe_reset
#define main lighthouse_cli_main
#include "../../survive-cli.c"
#undef main
#undef survive_reset_lighthouse_positions

TEST(CLI, MenuResetRequiresOptInAndButtonDown) {
	SurviveContext ctx = {0};
	config_entry setting = {.tag = "cli-menu-reset", .type = CONFIG_UINT32, .numeric.i = 1};
	config_group temporary = {.config_entries = &setting, .write_lock = OGCreateMutex()};
	ctx.temporary_config_values = &temporary;
	SurviveObject so = {.ctx = &ctx};
	resets = 0;
	button_process(&so, SURVIVE_INPUT_EVENT_BUTTON_DOWN, SURVIVE_BUTTON_MENU, NULL, NULL);
	ASSERT_EQ(resets, 0);
	temporary.used_entries = 1;
	button_process(&so, SURVIVE_INPUT_EVENT_BUTTON_UP, SURVIVE_BUTTON_MENU, NULL, NULL);
	button_process(&so, SURVIVE_INPUT_EVENT_TOUCH_DOWN, SURVIVE_BUTTON_MENU, NULL, NULL);
	button_process(&so, SURVIVE_INPUT_EVENT_BUTTON_DOWN, SURVIVE_BUTTON_TRIGGER, NULL, NULL);
	ASSERT_EQ(resets, 0);
	button_process(&so, SURVIVE_INPUT_EVENT_BUTTON_DOWN, SURVIVE_BUTTON_MENU, NULL, NULL);
	ASSERT_EQ(resets, 1);
	OGDeleteMutex(temporary.write_lock);
	return 0;
}
