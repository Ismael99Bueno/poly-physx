#include "constraint_interface.hpp"
#include <algorithm>

#define TOLERANCE 1.e-3f

namespace phys
{
    float constraint_interface::stiffness() const { return m_stiffness; }
    float constraint_interface::dampening() const { return m_dampening; }

    void constraint_interface::stiffness(float stiffness) { m_stiffness = stiffness; }
    void constraint_interface::dampening(float dampening) { m_dampening = dampening; }

    std::array<float, 3> constraint_interface::constraint_grad(entity2D &e) const { return gradient(e, &constraint_interface::value); }
    std::array<float, 3> constraint_interface::constraint_grad_derivative(entity2D &e) const { return gradient(e, &constraint_interface::derivative); }
    std::array<float, 3> constraint_interface::gradient(entity2D &e, const constraint_fun &constraint) const
    {
        const float x = e.pos().x;
        e.translate({-TOLERANCE, 0.f});
        const float cx1 = constraint(*this);
        e.translate({2.f * TOLERANCE, 0.f});
        const float cx2 = constraint(*this);
        e.pos({x, e.pos().y});

        const float y = e.pos().y;
        e.translate({0.f, -TOLERANCE});
        const float cy1 = constraint(*this);
        e.translate({0.f, 2.f * TOLERANCE});
        const float cy2 = constraint(*this);
        e.pos({e.pos().x, y});

        const float angpos = e.angpos();
        e.angpos(e.angpos() - TOLERANCE);
        const float ca1 = constraint(*this);
        e.angpos(e.angpos() + 2.f * TOLERANCE);
        const float ca2 = constraint(*this);
        e.angpos(angpos);

        return {(cx2 - cx1) / (2.f * TOLERANCE), (cy2 - cy1) / (2.f * TOLERANCE), (ca2 - ca1) / (2.f * TOLERANCE)};
    }
}