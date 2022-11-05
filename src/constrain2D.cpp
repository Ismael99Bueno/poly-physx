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

    float constrain2D::constrain() const { return constrain(m_entities); }
    float constrain2D::constrain_dt() const { return constrain_dt(m_entities); };

    std::array<float, 3> constrain2D::constrain_grad(std::size_t index) const { return gradient(index, &constrain2D::constrain); }
    std::array<float, 3> constrain2D::constrain_grad_dt(std::size_t index) const { return gradient(index, &constrain2D::constrain_dt); }
    std::array<float, 3> constrain2D::gradient(std::size_t index,
                                               float (constrain2D::*constrain)(const std::vector<const_entity_ptr> &) const) const
    {
        const float x = m_grad_entities[index]->pos().x;
        m_grad_entities[index]->pos().x -= TOLERANCE;
        const float cx1 = (this->*constrain)(m_entities);
        m_grad_entities[index]->pos().x += 2.f * TOLERANCE;
        const float cx2 = (this->*constrain)(m_entities);
        m_grad_entities[index]->pos().x = x;

        const float y = m_grad_entities[index]->pos().y;
        m_grad_entities[index]->pos().y -= TOLERANCE;
        const float cy1 = (this->*constrain)(m_entities);
        m_grad_entities[index]->pos().y += 2.f * TOLERANCE;
        const float cy2 = (this->*constrain)(m_entities);
        m_grad_entities[index]->pos().y = y;

        const float angpos = m_grad_entities[index]->angpos();
        m_grad_entities[index]->angpos(m_grad_entities[index]->angpos() - TOLERANCE);
        const float ca1 = (this->*constrain)(m_entities);
        m_grad_entities[index]->angpos(m_grad_entities[index]->angpos() + 2.f * TOLERANCE);
        const float ca2 = (this->*constrain)(m_entities);
        m_grad_entities[index]->angpos(angpos);

        return {(cx2 - cx1) / (2.f * TOLERANCE), (cy2 - cy1) / (2.f * TOLERANCE), (ca2 - ca1) / (2.f * TOLERANCE)};
    }
}