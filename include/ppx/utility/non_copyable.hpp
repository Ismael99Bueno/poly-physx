#ifndef PPX_NON_COPYABLE_HPP
#define PPX_NON_COPYABLE_HPP

namespace ppx
{
class non_copyable
{
    non_copyable(const non_copyable &) = delete;
    non_copyable &operator=(const non_copyable &) = delete;

  protected:
    non_copyable() = default;
};
} // namespace ppx
#endif