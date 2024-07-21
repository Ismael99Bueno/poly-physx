#pragma once

#include "ppx/behaviours/behaviour2D.hpp"

namespace ppx
{
class interaction2D : public behaviour2D
{
  public:
    interaction2D(world2D &world, const std::string &name);
    virtual ~interaction2D() = default;

    virtual glm::vec3 force_pair(const state2D &state1, const state2D &state2) const = 0;

    glm::vec3 force(const state2D &state) const override final;

    bool add(body2D *body) override final;
    bool remove(std::size_t index) override final;

    float potential(const state2D &state, const glm::vec2 &position) const;
    float potential(const glm::vec2 &position) const;

    virtual float potential_energy_pair(const state2D &state1, const state2D &state2) const
    {
        return 0.f;
    }
    float potential_energy(const state2D &state) const override final;
    float potential_energy() const override final;

  private:
    mutable state2D m_unit;
};
} // namespace ppx
