#include "ppx/internal/pch.hpp"
#include "ppx/utility/indexable.hpp"

namespace ppx
{
indexable::indexable(const std::size_t index) : m_index(index)
{
}

std::size_t indexable::index() const
{
    return m_index;
}
void indexable::index(const std::size_t index)
{
    m_index = index;
}
} // namespace ppx