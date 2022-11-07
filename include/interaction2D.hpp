#ifndef INTERACTION2D_HPP
#define INTERACTION2D_HPP

#include "entity_ptr.hpp"
#include <unordered_set>

namespace physics
{
    class interaction2D
    {
    public:
        interaction2D(std::size_t allocations = 50);
        virtual vec2 acceleration(const entity2D &e1, const entity2D &e2) const = 0;

        void add(const entity_ptr &e);
        void remove(const entity_ptr &e);
        bool contains(const const_entity_ptr &e) const;

        const std::unordered_set<const_entity_ptr> &entities() const;

    private:
        std::unordered_set<const_entity_ptr> m_entities;
    };
}

#endif