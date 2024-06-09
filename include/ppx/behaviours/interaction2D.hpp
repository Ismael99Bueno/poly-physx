#pragma once

#include "ppx/behaviours/behaviour2D.hpp"

namespace ppx
{
class interaction2D : public behaviour2D
{
  public:
    interaction2D(world2D &world, const std::string &name);
    virtual ~interaction2D() = default;

    virtual glm::vec3 force_pair(const body2D &body1, const body2D &body2) const = 0;
    glm::vec3 force(const body2D &body) const override;

    bool add(body2D *body) override;
    bool remove(std::size_t index) override;

    float potential(const body2D &body, const glm::vec2 &position) const;
    float potential(const glm::vec2 &position) const;

    virtual float potential_energy_pair(const body2D &body1, const body2D &body2) const
    {
        return 0.f;
    }
    float potential_energy(const body2D &body) const override;
    float potential_energy() const override;

  private:
    mutable body2D m_unit;
};
} // namespace ppx
