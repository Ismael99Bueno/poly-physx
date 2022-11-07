#ifndef CONSTRAIN2D_HPP
#define CONSTRAIN2D_HPP

#include "entity_ptr.hpp"
#include <vector>
#include <array>

#define POS_PER_ENTITY 3

namespace physics
{
    class constrain2D
    {
    public:
        constrain2D(std::size_t allocations = 50);

        float value() const;
        float derivative() const;

        void add(const entity_ptr &e);
        void remove(const const_entity_ptr &e);
        bool contains(const const_entity_ptr &e) const;

    protected:
        virtual float constrain(const std::vector<const_entity_ptr> &entities) const = 0;
        virtual float constrain_derivative(const std::vector<const_entity_ptr> &entities) const = 0;

    private:
        std::vector<const_entity_ptr> m_entities;
        std::vector<entity_ptr> m_grad_entities;

        std::array<float, POS_PER_ENTITY> constrain_grad(const entity_ptr &e) const;
        std::array<float, POS_PER_ENTITY> constrain_grad_derivative(const entity_ptr &e) const;
        std::array<float, POS_PER_ENTITY> gradient(const entity_ptr &e,
                                                   float (constrain2D::*constrain)() const) const;

        friend class compeller2D;
    };
}

#endif