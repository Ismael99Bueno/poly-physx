#ifndef CONSTRAINT2D_HPP
#define CONSTRAINT2D_HPP
#include "ppx/internal/core.hpp"

#include "ppx/constraints/constraint_interface2D.hpp"
#include "ppx/entity2D_ptr.hpp"

namespace ppx
{
template <std::size_t N> class constraint2D : public constraint_interface2D
{
  public:
    constraint2D() = default;
    virtual ~constraint2D() = default;

    constraint2D(const std::array<entity2D_ptr, N> &entities) : m_grad_entities(entities)
    {
        copy_to_const_entities(entities);
    }

    constraint2D(const std::array<entity2D_ptr, N> &entities, float stiffness, float dampening)
        : constraint_interface2D(stiffness, dampening), m_grad_entities(entities)
    {
        copy_to_const_entities(entities);
    }

    void add_entities(const std::array<entity2D_ptr, N> &entities)
    {
        m_grad_entities = entities;
        copy_to_const_entities(entities);
    }

    float value() const override
    {
        return constraint(m_entities);
    }
    virtual bool validate() override
    {
        for (const_entity2D_ptr &e : m_entities)
            if (!e.validate())
                return false;
        for (entity2D_ptr &e : m_grad_entities)
            if (!e.validate())
                return false;
        return true;
    }

  private:
    std::array<const_entity2D_ptr, N> m_entities;
    std::array<entity2D_ptr, N> m_grad_entities;

    void copy_to_const_entities(const std::array<entity2D_ptr, N> &entities)
    {
        for (std::size_t i = 0; i < N; i++)
            m_entities[i] = entities[i];
    }

    virtual float constraint(const std::array<const_entity2D_ptr, N> &entities) const = 0;
    virtual float constraint_derivative(const std::array<const_entity2D_ptr, N> &entities) const = 0;

    float derivative() const override
    {
        return constraint_derivative(m_entities);
    }
    std::size_t size() const override
    {
        return N;
    }
    entity2D &operator[](std::size_t index) const override
    {
        return *m_grad_entities[index];
    }
};
} // namespace ppx

#endif