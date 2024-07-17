#pragma once

#include "ppx/collider/collider2D.hpp"
#include "ppx/internal/worldref.hpp"
#include "kit/utility/utils.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/container/hashable_tuple.hpp"

namespace ppx
{
class world2D;
class broad_phase2D : public worldref2D, public kit::toggleable, kit::non_copyable
{
  public:
    struct pair
    {
        pair(collider2D *collider1, collider2D *collider2) : collider1(collider1), collider2(collider2)
        {
        }
        pair() = default;
        collider2D *collider1;
        collider2D *collider2;
    };
    using ctuple = kit::non_commutative_tuple<collider2D *, collider2D *>;

    broad_phase2D(world2D &world);
    virtual ~broad_phase2D() = default;
    virtual const char *name() const;
    virtual void on_attach()
    {
    }

    // as metrics, add the amount of pairs that had to be updated in a single frame

    const std::vector<pair> &update_pairs();
    void flag_update(collider2D *collider);
    void clear_pending_updates();

    std::size_t new_pairs_count() const;
    std::size_t pending_updates() const;

    KIT_TOGGLEABLE_FINAL_DEFAULT_SETTER()

    void remove_pairs_containing(const collider2D *collider);
    const std::vector<pair> &pairs() const;

    specs::collision_manager2D::broad2D params;

  protected:
    bool is_potential_new_pair(collider2D **collider1, collider2D **collider2) const;

    std::vector<pair> m_pairs;
    std::unordered_set<ctuple> m_unique_pairs;
    std::size_t m_new_pairs_count;

  private:
    virtual void update_pairs(const std::vector<collider2D *> &to_update) = 0;
    void remove_outdated_pairs();

    std::vector<collider2D *> m_to_update;

    std::vector<pair> m_last_pairs;
};

} // namespace ppx
