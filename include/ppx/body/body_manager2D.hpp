#pragma once

#include "rk/integration/state.hpp"
#include "ppx/manager2D.hpp"
#include "ppx/body/body2D.hpp"
#include "ppx/common/alias.hpp"

namespace ppx
{
class body_manager2D final : public manager2D<body2D>
{
  public:
    body2D *add(const body2D::specs &spc = {});

    using manager2D<body2D>::operator[];
    std::vector<const body2D *> operator[](const aabb2D &aabb) const;
    std::vector<body2D *> operator[](const aabb2D &aabb);

    std::vector<const body2D *> operator[](const glm::vec2 &point) const;
    std::vector<body2D *> operator[](const glm::vec2 &point);

    using manager2D<body2D>::remove;
    bool remove(std::size_t index) override;

    bool checksum() const;
    bool all_asleep() const;

    const std::vector<state2D> &states() const;

    specs::body_manager2D params;

  private:
    using manager2D<body2D>::manager2D;

    void gather_and_load_states(rk::state<float> &rkstate);
    void update_states(const std::vector<float> &posvels);

    void integrate_velocities(float ts);
    void integrate_positions(float ts);

    std::vector<float> load_velocities_and_forces() const;
    bool retrieve_data_from_states();

    std::vector<state2D> &mutable_states();

    std::vector<state2D> m_states;

    friend class world2D;
};
} // namespace ppx
