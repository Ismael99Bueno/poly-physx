#pragma once

#include "ppx/joints/joint_proxy2D.hpp"

namespace ppx
{
class world2D;
class spring2D : public kit::identifiable<>, public kit::indexable
{
  public:
    using ptr = kit::vector_ptr<spring2D>;
    using const_ptr = kit::const_vector_ptr<spring2D>;

    struct specs
    {
        joint_proxy2D::specs joint;
        float stiffness = 1.f;
        float damping = 0.f;
        float length = 0.f;

        std::uint32_t non_linear_terms = 0;
        float non_linear_contribution = 0.001f;

        static specs from_spring(const spring2D &sp);
    };

    spring2D(float stiffness = 1.f, float damping = 0.f, float length = 0.f, std::uint32_t non_linear_terms = 0,
             float non_linear_contribution = 0.001f);
    spring2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1 = glm::vec2(0.f),
             const glm::vec2 &anchor2 = glm::vec2(0.f), float stiffness = 1.f, float damping = 0.f, float length = 0.f,
             std::uint32_t non_linear_terms = 0, float non_linear_contribution = 0.001f);
    spring2D(const specs &spc);

    joint_proxy2D joint;
    float stiffness;
    float damping;
    float length;

    std::uint32_t non_linear_terms;
    float non_linear_contribution;

    world2D *world = nullptr;

    const_ptr as_ptr() const;
    ptr as_ptr();

    glm::vec4 force() const;

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    void apply_force_to_bodies();

  private:
    glm::vec2 non_linear_displacement(const glm::vec2 &displacement) const;
};

} // namespace ppx
