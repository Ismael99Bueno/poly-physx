#ifndef PPX_UUID_HPP
#define PPX_UUID_HPP

#include <cstdint>

namespace ppx
{
class uuid
{
  public:
    uuid();
    uuid(std::uint64_t uuid);

    operator std::uint64_t() const;

  private:
    std::uint64_t m_uuid;
};

bool operator==(const uuid &id1, const uuid &id2);
bool operator!=(const uuid &id1, const uuid &id2);

} // namespace ppx

namespace std
{
template <> struct hash<ppx::uuid>
{
    size_t operator()(const ppx::uuid &id) const;
};
} // namespace std

#endif