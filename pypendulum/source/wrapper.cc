// Wrapper code for our pendulum dynamics model.

// Some macros to disable particular errors in third party imports.
#if defined(__GNUC__)  // gcc
#define BEGIN_THIRD_PARTY_INCLUDES                                            \
  _Pragma("GCC diagnostic push")                       /* push */             \
      _Pragma("GCC diagnostic ignored \"-Wpedantic\"") /* disable pedantic */
#define END_THIRD_PARTY_INCLUDES _Pragma("GCC diagnostic pop")

#elif defined(__clang__)  // clang
#define BEGIN_THIRD_PARTY_INCLUDES                                  \
  _Pragma("clang diagnostic push")                       /* push */ \
      _Pragma("clang diagnostic ignored \"-Wpedantic\"") /* disable pedantic */
#define END_THIRD_PARTY_INCLUDES _Pragma("clang diagnostic pop")
#else
#define BEGIN_THIRD_PARTY_INCLUDES
#define END_THIRD_PARTY_INCLUDES
#endif

BEGIN_THIRD_PARTY_INCLUDES
#include <nanobind/nanobind.h>
END_THIRD_PARTY_INCLUDES

namespace py = nanobind;
namespace pendulum {}  // namespace pendulum

NB_MODULE(PY_MODULE_NAME, m) {
    (void)m;
}
