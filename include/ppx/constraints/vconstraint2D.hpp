#pragma once

#include "ppx/joints/joint2D.hpp"

namespace ppx
{
template <std::size_t LinDegrees, std::size_t AngDegrees>
concept LegalDegrees2D =
    LinDegrees + AngDegrees <= 3 && LinDegrees + AngDegrees > 0 && LinDegrees <= 2 && AngDegrees <= 1;

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
class vconstraint2D;

class vconstraint10_2D;
class vconstraint01_2D;
using vconstraint11_2D = vconstraint2D<1, 1>;
using vconstraint20_2D = vconstraint2D<2, 0>;
using vconstraint21_2D = vconstraint2D<2, 1>;

template <typename T>
concept VConstraint2D = requires() {
    requires Joint2D<T>;
    requires kit::DerivedFrom<T, vconstraint10_2D> || kit::DerivedFrom<T, vconstraint01_2D> ||
                 kit::DerivedFrom<T, vconstraint11_2D> || kit::DerivedFrom<T, vconstraint20_2D> ||
                 kit::DerivedFrom<T, vconstraint21_2D>;
};

template <std::size_t Degrees> struct degree_types
{
    using flat_t = glm::vec<Degrees, float>;
    using square_t = glm::mat<Degrees, Degrees, float>;
};
template <> struct degree_types<1>
{
    using flat_t = float;
    using square_t = float;
};

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
class vconstraint2D : public joint2D
{
  public:
    using flat_t = typename degree_types<LinDegrees + AngDegrees>::flat_t;
    using square_t = typename degree_types<LinDegrees + AngDegrees>::square_t;

    virtual ~vconstraint2D() = default;

    virtual flat_t constraint_velocity() const = 0;

    virtual void solve_velocities();
    virtual void startup();
    virtual void warmup();

  protected:
    using joint2D::joint2D;
    flat_t m_cumimpulse{0.f};

    glm::vec2 m_ganchor1;
    glm::vec2 m_ganchor2;

    glm::vec2 m_offset1;
    glm::vec2 m_offset2;

    square_t m_mass;

    virtual void update_constraint_data();

    virtual square_t inverse_mass() const = 0;

    virtual flat_t compute_constraint_impulse() const;
    virtual glm::vec2 compute_linear_impulse(const flat_t &cimpulse) const;

    void apply_linear_impulse(const glm::vec2 &linimpulse);
    void apply_angular_impulse(float angimpulse);
    void solve_velocities_clamped(const flat_t &min, const flat_t &max);

  private:
    float compute_angular_impulse(const flat_t &cimpulse) const;
    virtual void solve() override;
};

class vconstraint10_2D : public vconstraint2D<1, 0>
{
  protected:
    using vconstraint2D<1, 0>::vconstraint2D;

    glm::vec2 m_dir;

    virtual float inverse_mass() const override;
    virtual void update_constraint_data() override;

  private:
    virtual glm::vec2 direction() const = 0;
    glm::vec2 compute_linear_impulse(const float &impulse) const override;
};

class vconstraint01_2D : public vconstraint2D<0, 1>
{
  protected:
    using vconstraint2D<0, 1>::vconstraint2D;

    virtual float inverse_mass() const override;
};

} // namespace ppx
