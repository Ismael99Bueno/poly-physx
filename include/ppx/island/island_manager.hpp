#pragma once

#include "ppx/manager.hpp"
#include "ppx/island/island.hpp"

namespace ppx
{
class island_manager2D final : public contiguous_manager2D<island2D>
{
  public:
    using contiguous_manager2D::contiguous_manager2D;

    bool enabled() const;
    void enabled(bool enable);

    bool checksum() const;
    bool all_asleep() const;

    using contiguous_manager2D::remove;
    bool remove(std::size_t index);

    float sleep_energy_threshold(const island2D *island) const;

    specs::island_manager2D params;

  private:
    void solve_actuators(std::vector<state2D> &states);

    void solve_velocity_constraints(std::vector<state2D> &states);
    void solve_position_constraints(std::vector<state2D> &states);

    island2D *create_and_add();
    island2D *create_island_from_body(body2D *body);

    void try_split();
    void remove_invalid_and_gather_awake();

    bool split(island2D *island);
    void build_from_existing_simulation();

    bool m_enable = true;
    std::size_t m_remove_index = 0;
    std::vector<island2D *> m_awake_islands;

    friend class world2D;
    friend class body2D;
    friend class body_manager2D;
};
} // namespace ppx