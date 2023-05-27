#include "ppx/pch.hpp"
#include "ppx/constraint_interface2D.hpp"

#define TOLERANCE 1.e-3f

namespace ppx
{
constraint_interface2D::constraint_interface2D(const float stiffness, const float dampening)
    : m_stiffness(stiffness), m_dampening(dampening)
{
}

float constraint_interface2D::stiffness() const
{
    return m_stiffness;
}
float constraint_interface2D::dampening() const
{
    return m_dampening;
}

void constraint_interface2D::stiffness(float stiffness)
{
    m_stiffness = stiffness;
}
void constraint_interface2D::dampening(float dampening)
{
    m_dampening = dampening;
}

std::array<float, 3> constraint_interface2D::constraint_grad(entity2D &e) const
{
    return gradient(e, &constraint_interface2D::value);
}
std::array<float, 3> constraint_interface2D::constraint_grad_derivative(entity2D &e) const
{
    return gradient(e, &constraint_interface2D::derivative);
}
std::array<float, 3> constraint_interface2D::gradient(entity2D &e, const constraint_fun &constraint) const
{
    const glm::vec2 pos = e.pos();
    e.translate({-TOLERANCE, 0.f});
    const float cx1 = constraint(*this);
    e.translate({2.f * TOLERANCE, 0.f});
    const float cx2 = constraint(*this);
    e.pos(pos);

    e.translate({0.f, -TOLERANCE});
    const float cy1 = constraint(*this);
    e.translate({0.f, 2.f * TOLERANCE});
    const float cy2 = constraint(*this);
    e.pos(pos);

    const float angpos = e.angpos();
    e.angpos(e.angpos() - TOLERANCE);
    const float ca1 = constraint(*this);
    e.angpos(e.angpos() + 2.f * TOLERANCE);
    const float ca2 = constraint(*this);
    e.angpos(angpos);

    return {(cx2 - cx1) / (2.f * TOLERANCE), (cy2 - cy1) / (2.f * TOLERANCE), (ca2 - ca1) / (2.f * TOLERANCE)};
}
} // namespace ppx