#ifndef ENTITY2D_PTR_HPP
#define ENTITY2D_PTR_HPP

#include "entity2D.hpp"
#include <vector>

namespace ppx
{
    class const_entity2D_ptr
    {
    public:
        const_entity2D_ptr() = default;
        const_entity2D_ptr(const std::vector<entity2D> *buffer, std::size_t index = 0);

        std::size_t index() const;
        std::size_t id() const;

        const entity2D *raw() const;
        const entity2D *operator->() const;
        const entity2D &operator*() const;

        bool is_valid() const;
        bool try_validate();

        explicit operator bool() const;

    private:
        const std::vector<entity2D> *m_buffer = nullptr;
        std::size_t m_index = 0, m_id = 0;
    };

    bool operator==(const const_entity2D_ptr &e1, const const_entity2D_ptr &e2);
    bool operator!=(const const_entity2D_ptr &e1, const const_entity2D_ptr &e2);

    class entity2D_ptr
    {
    public:
        entity2D_ptr() = default;
        entity2D_ptr(std::vector<entity2D> *buffer, std::size_t index = 0);

        std::size_t index() const;
        std::size_t id() const;

        entity2D *raw() const;
        entity2D *operator->() const;
        entity2D &operator*() const;

        bool is_valid() const;
        bool try_validate();

        explicit operator bool() const;
        operator const_entity2D_ptr() const;

    private:
        std::vector<entity2D> *m_buffer = nullptr;
        std::size_t m_index = 0, m_id = 0;
    };

    bool operator==(const entity2D_ptr &e1, const entity2D_ptr &e2);
    bool operator!=(const entity2D_ptr &e1, const entity2D_ptr &e2);
}

namespace std
{
    template <>
    struct hash<ppx::const_entity2D_ptr>
    {
        size_t operator()(const ppx::const_entity2D_ptr &key) const;
    };

    template <>
    struct hash<ppx::entity2D_ptr>
    {
        size_t operator()(const ppx::entity2D_ptr &key) const;
    };
}

#endif