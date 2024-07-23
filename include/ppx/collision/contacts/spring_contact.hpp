#pragma once

#include "ppx/collision/contacts/contact.hpp"

#ifdef _MSC_VER
#pragma warning(push) // Inheritance via dominance is intended
#pragma warning(disable : 4250)
#endif

namespace ppx
{
class spring_contact2D final : public contact2D, public actuator2D
{
  public:
    static inline float rigidity = 2000.f;
    static inline float max_normal_damping = 8.5f;
    static inline float max_tangent_damping = 8.5f;

    spring_contact2D(world2D &world, const collision2D *collision, std::size_t manifold_index);

    void update(const collision2D *collision, std::size_t manifold_index) override;
    glm::vec3 compute_force(const state2D &state1, const state2D &state2) const override;

  private:
    float m_normal_damping;
    float m_tangent_damping;

    void compute_parameters();
};
} // namespace ppx

#ifdef _MSC_VER
#pragma warning(pop)
#endif