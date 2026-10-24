#include "test_case.h"

int PoserBaryCentricSVD(SurviveObject *so, PoserData *pd);

TEST(BarycentricSVD, WaitForSensorGeometry) {
	SurviveContext ctx = {0};
	SurviveObject so = {.ctx = &ctx};
	PoserData pd = {.pt = POSERDATA_IMU};
	PoserBaryCentricSVD(&so, &pd);
	ASSERT_EQ(*survive_object_plugin_data(&so, PoserBaryCentricSVD), 0);
	LinmathPoint3d points[] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	so.sensor_ct = 4;
	so.sensor_locations = (FLT *)points;
	so.has_sensor_locations = true;
	PoserBaryCentricSVD(&so, &pd);
	ASSERT_EQ((*survive_object_plugin_data(&so, PoserBaryCentricSVD) != 0), true);
	pd.pt = POSERDATA_DISASSOCIATE;
	PoserBaryCentricSVD(&so, &pd);
	ASSERT_EQ(*survive_object_plugin_data(&so, PoserBaryCentricSVD), 0);
	free(so.PluginDataEntries);
	return 0;
}
