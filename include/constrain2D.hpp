#ifndef CONSTRAIN2D_HPP
#define CONSTRAIN2D_HPP

#include "entity_ptr.hpp"
#include <vector>

namespace physics
{
    class constrain2D
    {
    public:
        constrain2D(std::size_t allocations = 50);

        virtual float constrain(const std::vector<const_entity_ptr> &m_entities) const = 0;
        virtual std::vector<float> constrain_grad(const std::vector<const_entity_ptr> &m_entities) const = 0;
        virtual std::vector<float> constrain_grad_dt(const std::vector<const_entity_ptr> &m_entities) const = 0;

        void add(const const_entity_ptr &e);
        void remove(const const_entity_ptr &e);
        bool contains(const const_entity_ptr &e) const;

    private:
        std::vector<const_entity_ptr> m_entities;
    };
}

#endif