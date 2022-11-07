#include "constrain2D.hpp"
#include <algorithm>

#define TOLERANCE 1.e-1f

namespace physics
{
    constrain2D::constrain2D(const std::size_t allocations) { m_entities.reserve(allocations); }

    void constrain2D::add(const entity_ptr &e)
    {
        m_entities.push_back(e);
        m_grad_entities.push_back(e);
    }

    void constrain2D::remove(const const_entity_ptr &e)
    {
        m_entities.erase(std::remove(m_entities.begin(), m_entities.end(), e), m_entities.end());
        m_grad_entities.erase(std::remove(m_grad_entities.begin(), m_grad_entities.end(), e), m_grad_entities.end());
    }

    bool constrain2D::contains(const const_entity_ptr &e) const
    {
        return std::find(m_entities.begin(), m_entities.end(), e) != m_entities.end();
    }

    float constrain2D::value() const { return constrain(m_entities); }
    float constrain2D::derivative() const { return constrain_derivative(m_entities); };

    std::array<float, 3> constrain2D::constrain_grad(const entity_ptr &e) const { return gradient(e, &constrain2D::value); }
    std::array<float, 3> constrain2D::constrain_grad_derivative(const entity_ptr &e) const { return gradient(e, &constrain2D::derivative); }
    std::array<float, 3> constrain2D::gradient(const entity_ptr &e,
                                               float (constrain2D::*constrain)() const) const
    {
        const float x = e->pos().x;
        e->pos().x -= TOLERANCE;
        const float cx1 = (this->*constrain)();
        e->pos().x += 2.f * TOLERANCE;
        const float cx2 = (this->*constrain)();
        e->pos().x = x;

        const float y = e->pos().y;
        e->pos().y -= TOLERANCE;
        const float cy1 = (this->*constrain)();
        e->pos().y += 2.f * TOLERANCE;
        const float cy2 = (this->*constrain)();
        e->pos().y = y;

        const float angpos = e->angpos();
        e->angpos(e->angpos() - TOLERANCE);
        const float ca1 = (this->*constrain)();
        e->angpos(e->angpos() + 2.f * TOLERANCE);
        const float ca2 = (this->*constrain)();
        e->angpos(angpos);

        return {(cx2 - cx1) / (2.f * TOLERANCE), (cy2 - cy1) / (2.f * TOLERANCE), (ca2 - ca1) / (2.f * TOLERANCE)};
    }
}