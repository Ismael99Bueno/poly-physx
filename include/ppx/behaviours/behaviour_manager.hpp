#pragma once

#include "kit/memory/ptr/scope.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/events/event.hpp"
#include "ppx/behaviours/behaviour.hpp"
#include "ppx/manager.hpp"

#include <vector>

namespace ppx
{
class behaviour_manager2D final : public id_contiguous_manager2D<kit::scope<behaviour2D>>
{
  public:
    template <kit::DerivedFrom<behaviour2D> T, class... BehaviourArgs> T *add(BehaviourArgs &&...args)
    {
        auto bhv = kit::make_scope<T>(world, std::forward<BehaviourArgs>(args)...);
        T *ptr = bhv.get();
        process_addition(std::move(bhv));
        return ptr;
    }

    template <kit::DerivedFrom<behaviour2D> T> T *from_name(const std::string &name) const
    {
        return dynamic_cast<T *>(from_name<behaviour2D>(name));
    }

    using id_contiguous_manager2D<kit::scope<behaviour2D>>::remove;
    bool remove(std::size_t index) override;

  private:
    using id_contiguous_manager2D<kit::scope<behaviour2D>>::id_contiguous_manager2D;

    void on_body_removal_validation(body2D *body);
    void load_forces(std::vector<state2D> &states);

    void process_addition(kit::scope<behaviour2D> &&bhv);
    friend class world2D;
};

template <> behaviour2D *behaviour_manager2D::from_name(const std::string &name) const;
} // namespace ppx
