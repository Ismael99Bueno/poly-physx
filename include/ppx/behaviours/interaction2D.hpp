#ifndef PPX_INTERACTION2D_HPP
#define PPX_INTERACTION2D_HPP

#include "ppx/behaviours/behaviour2D.hpp"

namespace ppx
{
class interaction2D : public behaviour2D
{
  public:
    using behaviour2D::behaviour2D;
    virtual ~interaction2D() = default;

    virtual std::pair<glm::vec2, float> force_pair(const body2D &bd1, const body2D &bd2) const = 0;
    std::pair<glm::vec2, float> force(const body2D &bd) const override;

    float potential(const body2D &bd, const glm::vec2 &position) const;
    float potential(const glm::vec2 &position) const;

    virtual float potential_energy_pair(const body2D &bd1, const body2D &bd2) const
    {
        return 0.f;
    }
    float potential_energy(const body2D &bd) const override;
    float potential_energy() const override;

  private:
    mutable body2D m_unit;
};
} // namespace ppx
#endif