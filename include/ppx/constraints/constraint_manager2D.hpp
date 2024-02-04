#pragma once

#include "ppx/constraints/contact_constraint2D.hpp"
#include "ppx/manager2D.hpp"
#include "kit/container/hashable_tuple.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/events/event.hpp"

namespace ppx
{
class constraint_manager2D final : public manager2D<kit::scope<constraint2D>>
{
  public:
    using manager2D<kit::scope<constraint2D>>::manager2D;

    struct
    {
        kit::event<constraint2D *> on_addition;
        kit::event<const constraint2D &> on_removal;
    } events;

    std::uint32_t iterations = 10;
    bool warmup = true;
    bool baumgarte_correction = true;

    float baumgarte_coef = 0.1f;
    float baumgarte_threshold = 0.05f;

    template <kit::DerivedFrom<constraint2D> T, class... ConstraintArgs> T *add(ConstraintArgs &&...args)
    {
        static_assert(!std::is_same_v<T, contact_constraint2D> || !std::is_same_v<T, friction_constraint2D>,
                      "Cannot add a contact or friction constraint directly");

        auto ctr = kit::make_scope<T>(world, std::forward<ConstraintArgs>(args)...);
        T *ptr = ctr.get();
        KIT_ASSERT_ERROR(ptr->valid(), "The constraint must be valid before it can be added into the simulation")

        process_addition(std::move(ctr));
        return ptr;
    }

    using manager2D<kit::scope<constraint2D>>::remove;
    bool remove(std::size_t index) override;

    using manager2D<kit::scope<constraint2D>>::operator[];
    std::vector<const constraint2D *> operator[](const std::vector<kit::uuid> &ids) const;
    std::vector<constraint2D *> operator[](const std::vector<kit::uuid> &ids);

    void validate();

  private:
    const std::vector<collision2D> *m_collisions = nullptr;
    std::unordered_map<kit::commutative_tuple<kit::uuid, kit::uuid, std::size_t>, contact_constraint2D> m_contacts;

    void update_contacts();
    void process_addition(kit::scope<constraint2D> &&ctr);

    void delegate_collisions(const std::vector<collision2D> *collisions);

    void solve();

    friend class world2D;
    friend class constraint_driven_resolution2D;
};
} // namespace ppx
