#pragma once

#include "ppx/manager2D.hpp"
#include "ppx/island/island2D.hpp"

namespace ppx
{
class island_manager2D final : public manager2D<island2D>
{
  public:
    using manager2D::manager2D;

    float sleep_energy_threshold = 0.6f;
    float sleep_time_threshold = 1.f;
    bool enable_split = true;

    bool enabled() const;
    void enabled(bool enable);

    bool checksum() const;

    using manager2D::remove;
    bool remove(std::size_t index);
    void remove_invalid();

  private:
    bool m_enable = true;
    std::size_t m_island_to_split = 0;

    void solve();

    island2D *create();
    island2D *create_island_from_body(body2D *body);

    void try_split(std::uint32_t max_tries);
    bool split(island2D *island);
    void build_from_existing_simulation();

    friend class world2D;
    friend class body2D;
    friend class body_manager2D;
};
} // namespace ppx