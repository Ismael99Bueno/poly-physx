#pragma once

#include "ppx/constraints/constraint2D.hpp"
#include "ppx/entities/specs2D.hpp"

namespace ppx
{
class distance_joint2D final : public constraint2D
{
  public:
    using ptr = kit::vector_ptr<distance_joint2D>;
    using const_ptr = kit::const_vector_ptr<distance_joint2D>;
    using specs = specs::distance_joint2D;

    distance_joint2D(world2D &world, const specs &spc);

    const_ptr as_ptr() const;
    ptr as_ptr();

    float min_distance;
    float max_distance;

    float constraint_value() const override;
    float constraint_velocity() const override;

    void startup() override;
    void solve() override;

  private:
    float inverse_mass() const override;
    glm::vec2 direction() const override;

    bool legal_length() const;

    float m_length;
};
} // namespace ppx
