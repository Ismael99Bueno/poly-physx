#ifndef PPX_CORE_HPP
#define PPX_CORE_HPP

#include "geo/core.hpp"
#include <memory>

#ifdef HAS_ALLOCATORS
#include "mem/stack_allocator.hpp"
#include "mem/block_allocator.hpp"

#include <vector>

namespace ppx
{
    template <typename T>
    using stk_vector = std::vector<T, mem::stack_allocator<T>>;

    template <typename T>
    using blk_vector = std::vector<T, mem::block_allocator<T>>;

    template <typename T>
    using scope = std::unique_ptr<T, mem::block_deleter<T>>;

    template <typename T>
    using ref = std::shared_ptr<T>;

    template <typename T, class... Args>
    inline scope<T> make_scope(Args &&...args)
    {
        static mem::block_allocator<T> alloc; // I dont think static is even worth it
        T *buff = alloc.allocate(1);
        T *p = new (buff) T(std::forward<Args>(args)...);
        return scope<T>(p);
    }

    template <typename T, class... Args>
    inline ref<T> make_ref(Args &&...args)
    {
        static mem::block_allocator<T> alloc;
        static mem::block_deleter<T> deleter;
        T *buff = alloc.allocate(1);
        T *p = new (buff) T(std::forward<Args>(args)...);
        return ref<T>(p, deleter);
    }
}
#else

namespace ppx
{
    template <typename T>
    using stk_vector = std::vector<T>;

    template <typename T>
    using blk_vector = std::vector<T>;

    template <typename T>
    using scope = std::unique_ptr<T>;

    template <typename T>
    using ref = std::shared_ptr<T>;

    template <typename T, class... Args>
    inline scope<T> make_scope(Args &&...args) { return std::make_unique<T>(std::forwad<Args>(args)...); }

    template <typename T, class... Args>
    inline ref<T> make_ref(Args &&...args) { return std::make_shared<T>(std::forward<Args>(args)...); }
}

#endif

#endif