#ifndef PPX_CONSTRAINT_MANAGER2D_HPP
#define PPX_CONSTRAINT_MANAGER2D_HPP

#include "ppx/events/world_events.hpp"
#include "ppx/constraints/contact_constraint2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/container/stack_vector.hpp"
#include "kit/memory/scope.hpp"
#include <vector>
#include <functional>
#include <memory>
#include <type_traits>

namespace ppx
{
class world2D;
class constraint_manager2D final : kit::non_copyable
{
  public:
    constraint_manager2D(world2D &world);

    template <typename T, class... ConstraintArgs> T *add(ConstraintArgs &&...args)
    {
        static_assert(std::is_base_of_v<constraint2D, T>, "Constraint must inherit from constraint2D!");
        auto ctr = kit::make_scope<T>(std::forward<ConstraintArgs>(args)...);
        T *ptr = ctr.get();

        m_constraints.push_back(std::move(ctr));
        ptr->world = &m_world;

        KIT_ASSERT_ERROR(ptr->valid(), "The constraint must be valid before it can be added into the simulation")
        m_events.on_constraint_addition(ptr);
        return ptr;
    }

    bool remove(std::size_t index);
    bool remove(const constraint2D *ctr);
    bool remove(kit::uuid id);

    auto begin() const
    {
        return m_constraints.begin();
    }
    auto end() const
    {
        return m_constraints.end();
    }

    auto begin()
    {
        return m_constraints.begin();
    }
    auto end()
    {
        return m_constraints.end();
    }

    const constraint2D &operator[](std::size_t index) const;
    constraint2D &operator[](std::size_t index);

    const constraint2D *from_id(kit::uuid id) const;
    constraint2D *from_id(kit::uuid id);

    std::vector<const constraint2D *> from_ids(const std::vector<kit::uuid> &ids) const;
    std::vector<constraint2D *> from_ids(const std::vector<kit::uuid> &ids);

    void delegate_collisions(const std::vector<collision2D> *collisions);
    void reset_delegated_collisions();

    std::size_t size() const;
    void clear();
    void validate();

    void solve();

  private:
    world2D &m_world;
    world_events &m_events;
    std::vector<kit::scope<constraint2D>> m_constraints;

    const std::vector<collision2D> *m_collisions;
    std::vector<contact_constraint2D> m_contacts;
};
} // namespace ppx

#endif