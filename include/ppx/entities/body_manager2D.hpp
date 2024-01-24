#pragma once

#include "ppx/entities/body2D.hpp"
#include "rk/integration/state.hpp"
#include "ppx/internal/worldref.hpp"

namespace ppx
{
class world2D;

class body_manager2D : public worldref2D
{
  public:
    body_manager2D(world2D &world);

    template <class... BodyArgs> body2D &add(BodyArgs &&...args)
    {
        body2D &body = m_bodies.emplace_back(world, std::forward<BodyArgs>(args)...);
        process_addition(body);
        return body;
    }

    auto begin() const
    {
        return m_bodies.begin();
    }
    auto end() const
    {
        return m_bodies.end();
    }

    auto begin()
    {
        return m_bodies.begin();
    }
    auto end()
    {
        return m_bodies.end();
    }

    const body2D &operator[](std::size_t index) const;
    body2D &operator[](std::size_t index);

    body2D::const_ptr ptr(std::size_t index) const;
    body2D::ptr ptr(std::size_t index);

    std::vector<body2D::const_ptr> operator[](const aabb2D &aabb) const;
    std::vector<body2D::ptr> operator[](const aabb2D &aabb);

    const body2D *operator[](const glm::vec2 &point) const;
    body2D *operator[](const glm::vec2 &point);

    const body2D *operator[](kit::uuid id) const;
    body2D *operator[](kit::uuid id);

    bool remove(std::size_t index);
    bool remove(const body2D &body);
    bool remove(kit::uuid id);

    void apply_impulse_and_persistent_forces();
    void reset_impulse_forces();
    void reset_simulation_forces();

    void send_data_to_state(rk::state<float> &state);
    void retrieve_data_from_state_variables(const std::vector<float> &vars_buffer);

    void prepare_constraint_velocities();

    std::size_t size() const;
    void clear();
    void validate();

  private:
    std::vector<body2D> m_bodies;

    void process_addition(body2D &body);
};
} // namespace ppx
