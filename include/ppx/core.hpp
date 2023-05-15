#ifndef PPX_CORE_HPP
#define PPX_CORE_HPP

#include "geo/core.hpp"

#ifdef HAS_ALLOCATORS
#include "mem/stack_allocator.hpp"
#include "mem/block_allocator.hpp"

namespace ppx
{
    template <typename T>
    using stack_alloc = mem::stack_allocator<T>;

    template <typename T>
    using block_alloc = mem::block_allocator<T>;
}
#else

#include <memory>
namespace ppx
{
    template <typename T>
    using stack_alloc = std::allocator<T>;

    template <typename T>
    using block_alloc = std::allocator<T>;
}

#endif

#endif