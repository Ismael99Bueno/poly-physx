#ifndef ENTITY2D_SET_HPP
#define ENTITY2D_SET_HPP

#include "entity2D_ptr.hpp"

namespace phys
{
    class entity2D_set
    {
    public:
        entity2D_set(std::size_t allocations = 50);

        void validate();

        void include(const const_entity2D_ptr &e);
        void exclude(const entity2D &e);
        bool contains(const entity2D &e) const;
        float kinetic_energy() const;
        void clear();
        std::size_t size() const;

        const std::vector<const_entity2D_ptr> &entities() const;

    protected:
        std::vector<const_entity2D_ptr> m_entities;
    };
}

#endif