#pragma once

#include "kit/memory/scope.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/utility/type_constraints.hpp"
#include "ppx/events/world_events.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/internal/worldref.hpp"

#include <vector>

namespace ppx
{
class world2D;
class behaviour_manager2D : kit::non_copyable, public worldref2D
{
  public:
    behaviour_manager2D(world2D &world);

    template <kit::DerivedFrom<behaviour2D> T, class... BehaviourArgs> T *add(BehaviourArgs &&...args)
    {
        auto bhv = kit::make_scope<T>(world, std::forward<BehaviourArgs>(args)...);
#ifdef DEBUG
        for (const auto &old : m_behaviours)
        {
            KIT_ASSERT_ERROR(
                *old != *bhv,
                "Cannot add a behaviour with a name that already exists. Behaviour names act as identifiers")
        }
#endif
        T *ptr = bhv.get();

        m_behaviours.push_back(std::move(bhv));
        m_events.on_behaviour_addition(ptr);
        return ptr;
    }

    template <kit::DerivedFrom<behaviour2D> T> T *from_name(const std::string &name) const
    {
        return dynamic_cast<T *>(from_name<behaviour2D>(name));
    }

    auto begin() const
    {
        return m_behaviours.begin();
    }
    auto end() const
    {
        return m_behaviours.end();
    }

    auto begin()
    {
        return m_behaviours.begin();
    }
    auto end()
    {
        return m_behaviours.end();
    }

    const behaviour2D &operator[](std::size_t index) const;
    behaviour2D &operator[](std::size_t index);

    const behaviour2D *operator[](const std::string &name) const;
    behaviour2D *operator[](const std::string &name);

    bool remove(std::size_t index);
    bool remove(const behaviour2D *bhv);
    bool remove(const std::string &name);

    std::size_t size() const;
    void clear();
    void validate();
    void apply_forces();

  private:
    world_events &m_events;
    std::vector<kit::scope<behaviour2D>> m_behaviours;
};

template <> behaviour2D *behaviour_manager2D::from_name(const std::string &name) const;
} // namespace ppx
