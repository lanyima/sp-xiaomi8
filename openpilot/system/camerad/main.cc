#include "system/camerad/cameras/camera_common.h"

#include <cassert>
#include <cerrno>
#include <cstring>
#include <sched.h>
#include <unistd.h>

#include "common/params.h"
#include "common/util.h"

// Pin current process to big core 6 (Kryo 385 Gold / A75)
// Retries on failure — launcher fork timing can cause transient EINVAL/ESRCH
static void pin_to_big_core() {
  for (int attempt = 0; attempt < 3; attempt++) {
    int ret = util::set_core_affinity({6});
    if (ret == 0) {
      fprintf(stderr, "set_core_affinity({6}) OK (attempt %d)\n", attempt);
      return;
    }
    int err = errno;
    fprintf(stderr, "set_core_affinity({6}) failed attempt %d: ret=%d errno=%d (%s)\n",
            attempt, ret, err, strerror(err));

    // Try alternative: sched_setaffinity with pid=0 (current thread shorthand)
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(6, &cpus);
    if (sched_setaffinity(0, sizeof(cpus), &cpus) == 0) {
      fprintf(stderr, "sched_setaffinity(0, {6}) fallback OK\n");
      return;
    }
    err = errno;
    fprintf(stderr, "sched_setaffinity(0, {6}) fallback failed: errno=%d (%s)\n", err, strerror(err));

    usleep(100000);  // 100ms before retry
  }
  fprintf(stderr, "WARNING: could not pin to core 6 after retries, continuing on default cores\n");
}

int main(int argc, char *argv[]) {
  pin_to_big_core();

  camerad_thread();
  return 0;
}
