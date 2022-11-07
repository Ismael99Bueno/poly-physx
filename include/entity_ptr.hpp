#ifndef ENTITY_PTR_HPP
#define ENTITY_PTR_HPP

#include "entity2D.hpp"
#include <vector>

namespace physics
{
    class const_entity_ptr
    {
    public:
        const_entity_ptr() = default;
        const_entity_ptr(const std::vector<entity2D> &buffer, std::size_t index = 0);

        std::size_t index() const;
        const entity2D *operator->() const;
        const entity2D &operator*() const;

        explicit operator bool() const;

    private:
        const std::vector<entity2D> *m_buffer = nullptr;
        std::size_t m_index = 0;

        friend struct std::hash<physics::const_entity_ptr>;
    };

    bool operator==(const const_entity_ptr &e1, const const_entity_ptr &e2);
    bool operator!=(const const_entity_ptr &e1, const const_entity_ptr &e2);

    class entity_ptr
    {
    public:
        entity_ptr() = default;
        entity_ptr(std::vector<entity2D> &buffer, std::size_t index = 0);

        std::size_t index() const;
        entity2D *operator->() const;
        entity2D &operator*() const;

        explicit operator bool() const;
        operator const_entity_ptr() const;

    private:
        std::vector<entity2D> *m_buffer = nullptr;
        std::size_t m_index = 0;

        friend struct std::hash<physics::entity_ptr>;
    };

    bool operator==(const entity_ptr &e1, const entity_ptr &e2);
    bool operator!=(const entity_ptr &e1, const entity_ptr &e2);
}

namespace std
{
    template <>
    struct hash<physics::const_entity_ptr>
    {
        size_t operator()(const physics::const_entity_ptr &key) const;
    };

    template <>
    struct hash<physics::entity_ptr>
    {
        size_t operator()(const physics::entity_ptr &key) const;
    };
}

#endif