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

  private:
    using manager2D<body2D>::manager2D;

    allocator<body2D> m_allocator;

    void apply_instant_and_persistent_forces();
    void reset_instant_forces();
    void reset_simulation_forces();

    void send_data_to_state(rk::state<float> &state);
    void retrieve_data_from_state_variables(const std::vector<float> &vars_buffer);

    void prepare_constraint_states();

    friend class world2D;
};
} // namespace ppx
