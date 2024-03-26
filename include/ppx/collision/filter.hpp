#pragma once

#include <cstdint>
#include <vector>

namespace ppx
{
struct filter
{
    static inline constexpr std::uint32_t MAX_GROUPS = 32;

    template <std::uint32_t GIndex, std::uint32_t... GIndices>
        requires(GIndex < 32)
    static inline constexpr std::uint32_t CGROUP = (1 << GIndex) | CGROUP<GIndices...>;

    template <std::uint32_t GIndex>
        requires(GIndex < 32)
    static inline constexpr std::uint32_t CGROUP<GIndex> = 1 << GIndex;
    static inline constexpr std::uint32_t ALL = 0xFFFFFFFF;

    std::uint32_t cgroups = CGROUP<0>;
    std::uint32_t collides_with = CGROUP<0>;

    static std::uint32_t group(const std::uint32_t group)
    {
        return 1 << group;
    }
    static std::uint32_t group(const std::vector<std::uint32_t> &groups)
    {
        std::uint32_t cgroup = 0;
        for (auto filter : groups)
        {
            KIT_ASSERT_ERROR(filter < 32, "Group must be less than 32");
            cgroup |= 1 << filter;
        }
        return cgroup;
    }
};
} // namespace ppx