#ifndef PPX_INDEXABLE_HPP
#define PPX_INDEXABLE_HPP

#include <cstddef>

namespace ppx
{
class indexable
{
  public:
    indexable() = default;
    indexable(std::size_t index);

    std::size_t index() const;
    void index(std::size_t index);

  private:
    std::size_t m_index = 0;
};
} // namespace ppx

#endif