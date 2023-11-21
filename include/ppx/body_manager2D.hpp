#ifndef PPX_BODY_MANAGER2D_HPP
#define PPX_BODY_MANAGER2D_HPP

#include "ppx/body2D.hpp"

namespace ppx
{
class world2D;

class body_manager2D
{
  public:
    body_manager2D(world2D &world);

    template <class... BodyArgs> body2D::ptr add(BodyArgs &&...args)
    {
        body2D &body = m_bodies.emplace_back(std::forward<BodyArgs>(args)...);
        return process_addition(body);
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

    std::vector<body2D::const_ptr> operator[](const geo::aabb2D &aabb) const;
    std::vector<body2D::ptr> operator[](const geo::aabb2D &aabb);

    body2D::const_ptr operator[](const glm::vec2 &point) const;
    body2D::ptr operator[](const glm::vec2 &point);

    body2D::const_ptr from_id(kit::uuid id) const;
    body2D::ptr from_id(kit::uuid id);

    bool remove(std::size_t index);
    bool remove(const body2D &body);
    bool remove(kit::uuid id);

    void apply_added_forces();
    void reset_added_forces();
    void reset_simulation_forces();

    void send_data_to_state(rk::state &state);
    void retrieve_data_from_state_variables(const std::vector<float> &vars_buffer);

    std::size_t size() const;
    void clear();

  private:
    world2D &m_world;
    std::vector<body2D> m_bodies;

    body2D::ptr process_addition(body2D &body);
};
} // namespace ppx

#endif