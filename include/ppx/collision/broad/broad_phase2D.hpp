#pragma once

#include "ppx/collider/collider2D.hpp"
#include "ppx/internal/worldref.hpp"
#include "kit/utility/utils.hpp"
#include "kit/interface/toggleable.hpp"

namespace ppx
{
class world2D;
class broad_phase2D : public worldref2D, public kit::toggleable, kit::non_copyable
{
  public:
    using cpair = std::pair<collider2D *, collider2D *>;

    broad_phase2D(world2D &world);
    virtual ~broad_phase2D() = default;
    virtual const char *name() const;
    virtual void on_attach()
    {
    }

    // as metrics, add the amount of pairs that had to be updated in a single frame

    const std::vector<cpair> &update_pairs();
    void flag_update(collider2D *collider);
    void clear_pending_updates();

    const std::vector<cpair> &pairs() const;
    std::size_t pending_updates() const;

    KIT_TOGGLEABLE_FINAL_DEFAULT_SETTER()

    void try_create_pair(collider2D *collider1, collider2D *collider2);
    void remove_pairs_containing(const collider2D *collider);

    specs::collision_manager2D::broad2D params;

  private:
    virtual void update_pairs(const std::vector<collider2D *> &to_update) = 0;

    std::vector<cpair> m_pairs;
    std::vector<collider2D *> m_to_update;
    std::vector<bool> m_pair_flags;
};

} // namespace ppx
