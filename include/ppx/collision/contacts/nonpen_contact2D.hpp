#pragma once

#include "ppx/collision/contacts/nonpen_friction2D.hpp"
#include "ppx/collision/contacts/contact2D.hpp"

#ifdef _MSC_VER
#pragma warning(push) // Inheritance via dominance is intended
#pragma warning(disable : 4250)
#endif

namespace ppx
{
class nonpen_contact2D final : public contact2D, public pvconstraint2D<1, 0>
{
  public:
    nonpen_contact2D(world2D &world, const collision2D *collision, std::size_t manifold_index);

    float constraint_position() const override;
    float constraint_velocity() const override;

    void startup() override;
    void solve_velocities() override;
    void update(const collision2D *collision, std::size_t manifold_index) override;

    static inline specs::constraint2D::properties global_props{};

  private:
    float m_pntr_correction = 0.f;
    nonpen_friction2D m_friction_contact;

    bool m_has_friction;
    bool m_is_adjusting_positions = false;

    float m_init_ctr_vel = 0.f;

    void update_position_data() override;
    void warmup() override;
    glm::vec2 direction() const override;
};
} // namespace ppx

#ifdef _MSC_VER
#pragma warning(pop)
#endif
