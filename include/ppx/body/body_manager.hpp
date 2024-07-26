#pragma once

#include "rk/integration/state.hpp"
#include "ppx/manager.hpp"
#include "ppx/body/body.hpp"
#include "ppx/common/alias.hpp"

namespace ppx
{

struct link_accessor2D
{
    static body2D *&next(body2D *element)
    {
        return element->meta.next;
    }
    static body2D *&prev(body2D *element)
    {
        return element->meta.prev;
    }
};

class body_manager2D final : public linked_manager2D<body2D, link_accessor2D>
{
  public:
    body2D *add(const body2D::specs &spc = {});

    std::vector<const body2D *> operator[](const aabb2D &aabb) const;
    std::vector<body2D *> operator[](const aabb2D &aabb);

    std::vector<const body2D *> operator[](const glm::vec2 &point) const;
    std::vector<body2D *> operator[](const glm::vec2 &point);

    void remove(body2D *body) override;

    bool checksum() const;
    bool all_asleep() const;

    const std::vector<state2D> &states() const;

    specs::body_manager2D params;

  private:
    using linked_manager2D<body2D, link_accessor2D>::linked_manager2D;

    void gather_and_load_states(rk::state<float> &rkstate);
    void update_states(const std::vector<float> &posvels);

    void integrate_velocities(float ts);
    void integrate_positions(float ts);

    std::vector<float> load_velocities_and_forces() const;
    bool retrieve_data_from_states(const std::vector<float> &posvels);

    std::vector<state2D> &mutable_states();

    std::vector<state2D> m_states;
    std::vector<body2D *> m_bodies;

    friend class world2D;
};
} // namespace ppx
