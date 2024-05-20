#pragma once

#include "ppx/body/body2D.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/internal/worldref.hpp"
#include "ppx/collision/contacts/contact2D.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/container/hashable_tuple.hpp"
#include "kit/utility/type_constraints.hpp"

namespace ppx
{
class collision_resolution2D : public worldref2D, public kit::toggleable, kit::non_copyable
{
  public:
    collision_resolution2D(world2D &world);
    virtual ~collision_resolution2D() = default;

    virtual void remove_any_contacts_with(const collider2D *collider) = 0;

  protected:
    template <kit::DerivedFrom<contact2D> T> struct contact_manager : public worldref2D
    {
        using contact_map =
            std::unordered_map<kit::non_commutative_tuple<const collider2D *, const collider2D *, std::uint32_t>, T *>;

        contact_manager(world2D &world) : worldref2D(world)
        {
        }

        void remove_any_contacts_with(const collider2D *collider, contact_map &contacts)
        {
            for (auto it = contacts.begin(); it != contacts.end();)
            {
                T *contact = it->second;
                if (contact->collider1() == collider || contact->collider2() == collider)
                {
                    contact->collider1()->events.on_contact_exit(*contact);
                    contact->collider2()->events.on_contact_exit(*contact);
                    global_on_contact_exit(world, *contact);
                    contact->collider1()->body()->meta.remove_contact(contact);
                    contact->collider2()->body()->meta.remove_contact(contact);
                    contact->collider1()->meta.remove_contact(contact);
                    contact->collider2()->meta.remove_contact(contact);
                    allocator<T>::destroy(contact);
                    it = contacts.erase(it);
                }
                else
                    ++it;
            }
        }

        void update(const collision_detection2D::collision_map &collisions, contact_map &contacts)
        {
            KIT_PERF_FUNCTION()
            for (const auto &pair : collisions)
            {
                const collision2D &collision = pair.second;
                for (std::size_t i = 0; i < collision.manifold.size(); i++)
                {
                    const kit::non_commutative_tuple<const collider2D *, const collider2D *, std::uint32_t> hash{
                        collision.collider1, collision.collider2, collision.manifold[i].id.key};
                    const auto old_contact = contacts.find(hash);
                    if (old_contact != contacts.end())
                        old_contact->second->update(&collision, i);
                    else
                    {
                        T *contact = allocator<T>::create(world, &collision, i);
                        contacts.emplace(hash, contact);
                        collision.collider1->body()->meta.contacts.push_back(contact);
                        collision.collider2->body()->meta.contacts.push_back(contact);
                        collision.collider1->meta.contacts.push_back(contact);
                        collision.collider2->meta.contacts.push_back(contact);
                        collision.collider1->events.on_contact_enter(contact);
                        collision.collider2->events.on_contact_enter(contact);
                        global_on_contact_enter(world, contact);
                    }
                }
            }
            for (auto it = contacts.begin(); it != contacts.end();)
            {
                T *contact = it->second;
                if (!contact->recently_updated)
                {
                    contact->collider1()->events.on_contact_exit(*contact);
                    contact->collider2()->events.on_contact_exit(*contact);
                    global_on_contact_exit(world, *contact);
                    contact->collider1()->body()->meta.remove_contact(contact);
                    contact->collider2()->body()->meta.remove_contact(contact);
                    contact->collider1()->meta.remove_contact(contact);
                    contact->collider2()->meta.remove_contact(contact);
                    allocator<T>::destroy(contact);
                    it = contacts.erase(it);
                }
                else
                {
                    contact->recently_updated = false;
                    ++it;
                }
            }
        }
    };

    static void global_on_contact_enter(world2D &world, contact2D *contact);
    static void global_on_contact_exit(world2D &world, contact2D &contact);
    static void global_on_contact_pre_solve(world2D &world, contact2D *contact);
    static void global_on_contact_post_solve(world2D &world, contact2D *contact);

  private:
    virtual void on_attach()
    {
    }

    virtual void resolve_contacts(const collision_detection2D::collision_map &collisions) = 0;

    friend class collision_manager2D;
};
} // namespace ppx
