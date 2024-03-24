#pragma once

#include "kit/memory/scope.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/events/event.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/manager2D.hpp"

#include <vector>

namespace ppx
{
class behaviour_manager2D final : public idmanager2D<kit::scope<behaviour2D>>
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

    using idmanager2D<kit::scope<behaviour2D>>::remove;
    bool remove(std::size_t index) override;

  private:
    using idmanager2D<kit::scope<behaviour2D>>::idmanager2D;

    void on_body_removal_validation(const body2D *body);
    void apply_forces();

    void process_addition(kit::scope<behaviour2D> &&bhv);
    friend class world2D;
};

template <> behaviour2D *behaviour_manager2D::from_name(const std::string &name) const;
} // namespace ppx
