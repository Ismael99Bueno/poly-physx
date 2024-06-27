#pragma once

#include "ppx/constraints/vconstraint2D.hpp"

namespace ppx
{
template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
class pvconstraint2D;

template <typename T>
concept IPVConstraint2D = requires() {
    requires IConstraint2D<T>;
    requires kit::DerivedFrom<T, pvconstraint2D<1, 0>> || kit::DerivedFrom<T, pvconstraint2D<0, 1>> ||
                 kit::DerivedFrom<T, pvconstraint2D<1, 1>> || kit::DerivedFrom<T, pvconstraint2D<2, 0>> ||
                 kit::DerivedFrom<T, pvconstraint2D<2, 1>>;
};

template <typename T>
concept PVConstraint2D = Joint2D<T> && IPVConstraint2D<T>;

template <typename T>
concept Constraint2D = VConstraint2D<T> || PVConstraint2D<T>;

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
class pvconstraint2D : public vconstraint2D<LinDegrees, AngDegrees>
{
  public:
    using flat_t = typename degree_types<LinDegrees + AngDegrees>::flat_t;
    using square_t = typename degree_types<LinDegrees + AngDegrees>::square_t;

    using vconstraint2D<LinDegrees, AngDegrees>::vconstraint2D;

    virtual bool solve_positions() override;
    virtual flat_t constraint_position() const = 0;

  protected:
    flat_t m_c{0.f};

    virtual void update_constraint_data() override;
    virtual void update_position_data();

    flat_t compute_constraint_impulse() const override;
    flat_t compute_constraint_correction() const;

    glm::vec2 compute_linear_correction(const flat_t &ccorrection) const;
    float compute_angular_correction(const flat_t &ccorrection) const;

    void apply_linear_correction(const glm::vec2 &lincorrection);
    void apply_angular_correction(float angcorrection);
};

} // namespace ppx