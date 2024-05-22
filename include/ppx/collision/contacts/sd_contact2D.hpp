#pragma once

#include "ppx/collision/contacts/contact2D.hpp"

namespace ppx
{
class sd_contact2D final : public contact2D, public actuator2D
{
  public:
    static inline float rigidity = 2000.f;
    static inline float max_normal_damping = 8.5f;
    static inline float max_tangent_damping = 8.5f;

    sd_contact2D(world2D &world, const collision2D *collision, std::size_t manifold_index);

    void update(const collision2D *collision, std::size_t manifold_index) override;
    void solve() override;

  private:
    float m_normal_damping;
    float m_tangent_damping;

    void compute_parameters();
};
} // namespace ppx