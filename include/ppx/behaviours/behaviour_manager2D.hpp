#pragma once

#include "kit/memory/scope.hpp"
#include "kit/interface/non_copyable.hpp"
#include "ppx/events/world_events.hpp"
#include "ppx/behaviours/behaviour2D.hpp"

#include <vector>

namespace ppx
{
class world2D;
class behaviour_manager2D : kit::non_copyable
{
  public:
    behaviour_manager2D(world2D &world);
    world2D &world;

    template <typename T, class... BehaviourArgs> T *add(BehaviourArgs &&...args)
    {
        static_assert(std::is_base_of_v<behaviour2D, T>, "Type must inherit from behaviour2D! (Although it is "
                                                         "recommended to inherit from force2D or interaction2D)");
        auto bhv = kit::make_scope<T>(std::forward<BehaviourArgs>(args)...);
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
        m_behaviours.back()->world = &world;
        m_events.on_behaviour_addition(ptr);
        return ptr;
    }

    template <typename T> T *from_name(const std::string &name) const
    {
        static_assert(std::is_base_of_v<behaviour2D, T>, "Type must inherit from behaviour2D! (Although it is "
                                                         "recommended to inherit from force2D or interaction2D)");
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
