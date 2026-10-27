#define _GNU_SOURCE
#include <link.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int record_module(struct dl_phdr_info *info, size_t size, void *user) {
	const char *path = info->dlpi_name;
	// Include preload experiments too; core/plugin-only manifests cannot prove
	// which additional filtering or diagnostic implementation actually ran.
	if (path && *path)
		fprintf(user, "module\t%s\n", path);
	return 0;
}

// Inspect after tracking and recording have finished. This probe does not
// interpose any tracking function or depend on a particular tracker ABI.
__attribute__((destructor)) static void record_runtime(void) {
	const char *path = getenv("LH_RUNTIME_MANIFEST");
	if (!path) return;
	FILE *stream = fopen(path, "w");
	if (!stream) return;
	char executable[4096];
	ssize_t length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
	if (length >= 0) {
		executable[length] = 0;
		fprintf(stream, "executable\t%s\n", executable);
	}
	dl_iterate_phdr(record_module, stream);
	fclose(stream);
}
