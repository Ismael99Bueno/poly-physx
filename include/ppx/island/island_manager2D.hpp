#pragma once

#include "ppx/manager2D.hpp"
#include "ppx/island/island2D.hpp"

namespace ppx
{
class island_manager2D final : public manager2D<island2D>
{
  public:
    using manager2D::manager2D;

    bool enabled() const;
    void enabled(bool enable);

    bool checksum() const;
    bool all_asleep() const;

    using manager2D::remove;
    bool remove(std::size_t index);
    void remove_invalid();

    float sleep_energy_threshold(const island2D *island) const;

    specs::island_manager2D params;

  private:
    void gather_awake_islands();
    void solve_actuators(std::vector<state2D> &states);
    void solve_constraints(std::vector<state2D> &states);

    island2D *create_and_add();
    island2D *create_island_from_body(body2D *body);

    void try_split();
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