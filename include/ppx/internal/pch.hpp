#pragma once

#include <vector>
#include <utility>
#include <limits>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <typeinfo>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_FORCE_INTRINSICS
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include <memory>
#include <type_traits>
#include <array>
#include <cstdint>
#include <variant>
#include <thread>
#ifdef KIT_USE_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif
#include <random>
#include <optional>
#include "kit/debug/log.hpp"
#include "kit/profiling/perf.hpp"
