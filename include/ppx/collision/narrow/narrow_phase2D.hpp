#pragma once

#include "ppx/common/alias.hpp"
#include "ppx/collision/collision2D.hpp"
#include "ppx/collision/broad/broad_phase2D.hpp"
#include "geo/algorithm/intersection2D.hpp"
#include "kit/interface/toggleable.hpp"

namespace ppx
{
class collision_contacts2D;
class narrow_phase2D : public worldref2D, public kit::toggleable, kit::non_copyable
{
  public:
    using pair = broad_phase2D::pair;
    struct result
    {
        bool intersects = false;
        glm::vec2 mtv{0.f};
        manifold2D manifold;
        operator bool() const;
    };

    using worldref2D::worldref2D;

    virtual ~narrow_phase2D() = default;

    virtual result polygon_polygon(const polygon &poly1, const polygon &poly2) const = 0;
    virtual result circle_polygon(const circle &circ, const polygon &poly) const = 0;

    void update_contacts(const std::vector<pair> &pairs, collision_contacts2D *contacts);
    virtual const char *name() const;

    virtual void on_attach()
    {
    }

    KIT_TOGGLEABLE_FINAL_DEFAULT_SETTER()
    specs::collision_manager2D::narrow2D params;

  private:
    void update_contacts_st(const std::vector<pair> &pairs);
    void update_contacts_mt(const std::vector<pair> &pairs);

    collision2D generate_collision(collider2D *collider1, collider2D *collider2) const;
    void cc_narrow_collision_check(collider2D *collider1, collider2D *collider2, collision2D &collision) const;
    void cp_narrow_collision_check(collider2D *collider1, collider2D *collider2, collision2D &collision) const;
    void pp_narrow_collision_check(collider2D *collider1, collider2D *collider2, collision2D &collision) const;

    collision_contacts2D *m_contacts = nullptr;
};

} // namespace ppx