#pragma once

#include "ppx/constraints/vconstraint2D.hpp"

namespace ppx
{
template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
class pvconstraint2D;

using pvconstraint10_2D = pvconstraint2D<1, 0>;
using pvconstraint01_2D = pvconstraint2D<0, 1>;
using pvconstraint11_2D = pvconstraint2D<1, 1>;
using pvconstraint20_2D = pvconstraint2D<2, 0>;
using pvconstraint21_2D = pvconstraint2D<2, 1>;

template <typename T>
concept PVConstraint2D = requires() {
    requires Joint2D<T>;
    requires kit::DerivedFrom<T, pvconstraint10_2D> || kit::DerivedFrom<T, pvconstraint01_2D> ||
                 kit::DerivedFrom<T, pvconstraint11_2D> || kit::DerivedFrom<T, pvconstraint20_2D> ||
                 kit::DerivedFrom<T, pvconstraint21_2D>;
};

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
struct pvbase
{
    using type = vconstraint2D<LinDegrees, AngDegrees>;
};
template <> struct pvbase<1, 0>
{
    using type = vconstraint10_2D;
};
template <> struct pvbase<0, 1>
{
    using type = vconstraint01_2D;
};

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
class pvconstraint2D : public pvbase<LinDegrees, AngDegrees>::type
{
  public:
    using base_t = typename pvbase<LinDegrees, AngDegrees>::type;
    using flat_t = typename degree_types<LinDegrees + AngDegrees>::flat_t;
    using square_t = typename degree_types<LinDegrees + AngDegrees>::square_t;

    virtual ~pvconstraint2D() = default;

    virtual bool solve_positions();
    virtual flat_t constraint_position() const = 0;

  protected:
    using base_t::base_t;
    flat_t m_c{0.f};

    virtual void update_constraint_data() override;
    virtual void update_position_data();

  private:
    flat_t compute_constraint_impulse() const override;
    flat_t compute_constraint_correction() const;
    glm::vec2 compute_linear_correction(const flat_t &ccorrection) const;
    float compute_angular_correction(const flat_t &ccorrection) const;

    void solve() override;

    void apply_linear_correction(const glm::vec2 &lincorrection);
    void apply_angular_correction(float angcorrection);
};

} // namespace ppx