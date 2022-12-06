#ifndef CONSTRAINT2D_HPP
#define CONSTRAINT2D_HPP

#include "constraint_interface.hpp"
#include "entity_ptr.hpp"

namespace phys
{
    template <std::size_t N>
    class constraint2D : public constraint_interface
    {
    public:
        constraint2D() = default;
        constraint2D(const std::array<entity_ptr, N> &entities) : m_grad_entities(entities)
        {
            for (std::size_t i = 0; i < N; i++)
                m_entities[i] = entities[i];
        }

        void add_entities(const std::array<entity_ptr, N> &entities)
        {
            m_grad_entities = entities;
            for (std::size_t i = 0; i < N; i++)
                m_entities[i] = entities[i];
        }

        float value() const override { return constraint(m_entities); }
        bool try_validate() override
        {
            for (const_entity_ptr &e : m_entities)
                if (!e.try_validate())
                    return false;
            return true;
        }

    protected:
        std::array<const_entity_ptr, N> m_entities;

    private:
        std::array<entity_ptr, N> m_grad_entities;
        virtual float constraint(const std::array<const_entity_ptr, N> &entities) const = 0;
        virtual float constraint_derivative(const std::array<const_entity_ptr, N> &entities) const = 0;

        float derivative() const override { return constraint_derivative(m_entities); }
        std::size_t size() const override { return N; }
        entity2D &operator[](std::size_t index) const override { return *m_grad_entities[index]; }
    };
}

#endif