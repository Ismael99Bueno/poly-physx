#ifndef PPX_PCH
#define PPX_PCH

#include <vector>
#include <utility>
#include <limits>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <glm/geometric.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <memory>
#include <type_traits>
#include <array>
#include <cstdint>
#include <variant>
#ifdef PPX_MULTITHREADED
#include <thread>
#endif
#ifdef HAS_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif
#include <random>
#include <optional>
#ifdef HAS_DEBUG_LOG_TOOLS
#include "dbg/log.hpp"
#endif
#ifdef HAS_PROFILE_TOOLS
#include "perf/perf.hpp"
#endif
#include "ppx/core.hpp"

#endif