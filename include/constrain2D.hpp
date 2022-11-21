#ifndef CONSTRAIN2D_HPP
#define CONSTRAIN2D_HPP

#include "constrain_interface.hpp"

namespace phys
{
    template <std::size_t N>
    class constrain2D : public constrain_interface
    {
    public:
        constrain2D() = default;
        constrain2D(const std::array<entity_ptr, N> &entities) : m_grad_entities(entities)
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

        float value() const override { return constrain(m_entities); }

    protected:
        std::array<const_entity_ptr, N> m_entities;

    private:
        std::array<entity_ptr, N> m_grad_entities;
        virtual float constrain(const std::array<const_entity_ptr, N> &entities) const = 0;
        virtual float constrain_derivative(const std::array<const_entity_ptr, N> &entities) const = 0;

        float derivative() const override { return constrain_derivative(m_entities); }
        std::size_t size() const override { return N; }
        const entity_ptr &operator[](std::size_t index) const override { return m_grad_entities[index]; }
    };
}

#endif