#ifndef ENTITY_PTR_HPP
#define ENTITY_PTR_HPP

#include "entity2D.hpp"
#include <vector>

namespace phys
{
    class const_entity_ptr
    {
    public:
        const_entity_ptr() = default;
        const_entity_ptr(const std::vector<entity2D> &buffer, std::size_t index = 0);

        std::size_t index() const;
        const entity2D *raw() const;
        const entity2D *operator->() const;
        const entity2D &operator*() const;

        bool is_valid() const;
        bool try_validate();

        explicit operator bool() const;

    private:
        const std::vector<entity2D> *m_buffer = nullptr;
        std::size_t m_index = 0;
        std::uint64_t m_id = 0;

        friend struct std::hash<const_entity_ptr>;
    };

    bool operator==(const const_entity_ptr &e1, const const_entity_ptr &e2);
    bool operator!=(const const_entity_ptr &e1, const const_entity_ptr &e2);

    class entity_ptr
    {
    public:
        entity_ptr() = default;
        entity_ptr(std::vector<entity2D> &buffer, std::size_t index = 0);

        std::size_t index() const;
        entity2D *raw() const;
        entity2D *operator->() const;
        entity2D &operator*() const;

        bool is_valid() const;
        bool validate();

        explicit operator bool() const;
        operator const_entity_ptr() const;

    private:
        std::vector<entity2D> *m_buffer = nullptr;
        std::size_t m_index = 0;
        std::uint64_t m_id = 0;

        friend struct std::hash<entity_ptr>;
    };

    bool operator==(const entity_ptr &e1, const entity_ptr &e2);
    bool operator!=(const entity_ptr &e1, const entity_ptr &e2);
}

namespace std
{
    template <>
    struct hash<phys::const_entity_ptr>
    {
        size_t operator()(const phys::const_entity_ptr &key) const;
    };

    template <>
    struct hash<phys::entity_ptr>
    {
        size_t operator()(const phys::entity_ptr &key) const;
    };
}

#endif