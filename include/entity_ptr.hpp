#ifndef ENTITY_PTR_HPP
#define ENTITY_PTR_HPP

#include "entity2D.hpp"
#include <vector>

namespace physics
{
    class entity_ptr
    {
    public:
        entity_ptr() = delete;
        entity_ptr(const std::vector<entity2D> &vec, std::size_t index);

        const entity2D *operator->() const;
        const entity2D &operator*() const;

    private:
        const std::vector<entity2D> &m_vec;
        std::size_t m_index;
    };

    bool operator==(const entity_ptr &e1, const entity_ptr &e2);
}

#endif