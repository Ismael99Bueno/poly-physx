#ifndef ENTITY_SET_HPP
#define ENTITY_SET_HPP

#include "entity_ptr.hpp"
#include <unordered_set>

namespace phys
{
    class entity_set
    {
    public:
        entity_set(std::size_t allocations = 50);

        bool validate();

        void include(const const_entity_ptr &e);
        void exclude(const const_entity_ptr &e);
        bool contains(const const_entity_ptr &e) const;

        const std::unordered_set<const_entity_ptr> &entities() const;

    private:
        std::unordered_set<const_entity_ptr> m_entities;
    };
}

#endif