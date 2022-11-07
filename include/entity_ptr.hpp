#ifndef ENTITY_PTR_HPP
#define ENTITY_PTR_HPP

#include "entity2D.hpp"
#include <vector>

namespace physics
{
    class entity_ptr
    {
    public:
        entity_ptr(std::vector<entity2D> &vec, std::size_t index = 0);

        const entity2D *operator->() const;
        const entity2D &operator*() const;

        entity2D *operator->();
        entity2D &operator*();

        explicit operator bool() const;

    private:
        std::vector<entity2D> *m_vec = nullptr;
        std::size_t m_index = 0;
    };

    bool operator==(const entity_ptr &e1, const entity_ptr &e2);
    bool operator!=(const entity_ptr &e1, const entity_ptr &e2);
}

#endif