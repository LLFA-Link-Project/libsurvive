#ifndef SURVIVE_CALIBRATION_H
#define SURVIVE_CALIBRATION_H

#include <survive.h>

// Compare measured gravity in the device/base station frame with world up.
// This is independent of mounting angle; it is a diagnostic, not an acceptance
// threshold. Missing/invalid gravity is unavailable, not a zero error.
static inline FLT survive_calibration_up_error(const SurvivePose *local_to_world, const FLT *local_up) {
	FLT n = norm3d(local_up);
	if (!isfinite(n) || n == 0 || quatiszero(local_to_world->Rot))
		return NAN;
	LinmathPoint3d up;
	quatrotatevector(up, local_to_world->Rot, local_up);
	return atan2(hypot(up[0], up[1]), up[2]);
}

#endif
