#pragma once

#include "ppx/joints/joint2D.hpp"

namespace ppx
{
class spring2D : public joint2D, public worldref2D
{
  public:
    using ptr = kit::vector_ptr<spring2D>;
    using const_ptr = kit::const_vector_ptr<spring2D>;
    using specs = specs::spring2D;

    spring2D(world2D &world, const specs &spc);

    float stiffness;
    float damping;
    float length;

    std::uint32_t non_linear_terms;
    float non_linear_contribution;

    const_ptr as_ptr() const;
    ptr as_ptr();

    glm::vec4 force() const;

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    void solve() override;

  private:
    glm::vec2 non_linear_displacement(const glm::vec2 &displacement) const;
};

} // namespace ppx
