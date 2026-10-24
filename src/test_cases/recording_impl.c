// Exercise the private recorder lifecycle without starting USB drivers. Its
// configuration defaults are already registered by the core shared library.
#include <survive.h>
#undef SURVIVE_EXPORT_CONSTRUCTOR
#define SURVIVE_EXPORT_CONSTRUCTOR
#include "../survive_recording.c"
