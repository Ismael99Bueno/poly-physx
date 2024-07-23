#pragma once

#include "ppx/joints/joint.hpp"

namespace ppx
{
class constraint2D;
template <typename T>
concept IConstraint2D = kit::DerivedFrom<T, constraint2D>;

class constraint2D : virtual public joint2D
{
  public:
    constraint2D(const specs::constraint2D::properties &cprops = {});

    virtual void startup(std::vector<state2D> &states) = 0;
    virtual void solve_velocities() = 0;

    virtual bool solve_positions();

    bool is_constraint() const override final;

    specs::constraint2D::properties cprops() const;
    void cprops(const specs::constraint2D::properties &cprops);

    bool is_soft() const;
    void is_soft(bool is_soft);

    float frequency() const;
    void frequency(float frequency);

    float damping_ratio() const;
    void damping_ratio(float damping_ratio);

  protected:
    void fill_cprops(specs::constraint2D::properties &cprops) const;

    bool m_is_soft;
    float m_frequency;
    float m_damping_ratio;
};
} // namespace ppx