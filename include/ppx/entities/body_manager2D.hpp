#pragma once

#include "ppx/manager2D.hpp"
#include "ppx/entities/body2D.hpp"
#include "rk/integration/state.hpp"
#include "kit/events/event.hpp"

namespace ppx
{
class body_manager2D final : public manager2D<body2D>
{
  public:
    struct
    {
        kit::event<body2D &> on_addition;
        kit::event<const body2D &> on_early_removal;
        kit::event<std::size_t> on_late_removal;
    } events;

    body2D &add(const body2D::specs &spc = {});

    body2D::const_ptr ptr(std::size_t index) const;
    body2D::ptr ptr(std::size_t index);

    using manager2D<body2D>::operator[];
    std::vector<const body2D *> operator[](const aabb2D &aabb) const;
    std::vector<body2D *> operator[](const aabb2D &aabb);

    const body2D *operator[](const glm::vec2 &point) const;
    body2D *operator[](const glm::vec2 &point);

    using manager2D<body2D>::remove;
    bool remove(std::size_t index) override;

  private:
    using manager2D<body2D>::manager2D;

    void apply_impulse_and_persistent_forces();
    void reset_impulse_forces();
    void reset_simulation_forces();

    void send_data_to_state(rk::state<float> &state);
    void retrieve_data_from_state_variables(const std::vector<float> &vars_buffer);

    void prepare_constraint_velocities();
    void on_body_removal_validation();

    friend class world2D;
};
} // namespace ppx
