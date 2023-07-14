#ifndef PPX_IDENTIFIABLE_HPP
#define PPX_IDENTIFIABLE_HPP

#include "ppx/utility/uuid.hpp"

namespace ppx
{
class identifiable
{
  public:
    identifiable() = default;
    identifiable(uuid id);

    uuid id() const;
    void id(uuid id);

  private:
    uuid m_uuid;
};
} // namespace ppx

#endif