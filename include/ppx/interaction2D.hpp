#ifndef INTERACTION2D_HPP
#define INTERACTION2D_HPP
#include "ppx/core.hpp"

#include "ppx/behaviour2D.hpp"

namespace ppx
{
    class interaction2D : public behaviour2D
    {
    public:
        using behaviour2D::behaviour2D;
        virtual ~interaction2D() = default;

        virtual std::pair<glm::vec2, float> force_pair(const entity2D &e1, const entity2D &e2) const = 0;
        std::pair<glm::vec2, float> force(const entity2D &e) const override;

        float potential(const entity2D &e, const glm::vec2 &pos) const;
        float potential(const glm::vec2 &pos) const;

        virtual float potential_energy_pair(const entity2D &e1, const entity2D &e2) const { return 0.f; }
        float potential_energy(const entity2D &e) const override;
        float potential_energy() const override;

    private:
        mutable entity2D m_unit;

        interaction2D(const interaction2D &) = delete;
        interaction2D &operator=(const interaction2D &) = delete;
    };
}
#endif