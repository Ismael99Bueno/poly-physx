#ifndef PPX_EVENT_HPP
#define PPX_EVENT_HPP

#include <functional>
#include <vector>

namespace ppx
{
template <class... Ts> class event final
{
  public:
    event()
    {
        m_subscriptions.reserve(10);
    }

  private:
    using subscription = std::function<void(Ts...)>;

  public:
    event &operator+=(const subscription &s)
    {
        m_subscriptions.push_back(s);
        return *this;
    }
    event &operator-=(const subscription &s)
    {
        for (auto it = m_subscriptions.begin(); it != m_subscriptions.end(); ++it)
            if (*it == s)
            {
                m_subscriptions.erase(it);
                return *this;
            }
        return *this;
    }
    void operator()(Ts &&...args) const
    {
        for (const auto &sub : m_subscriptions)
            sub(std::forward<Ts>(args)...);
    }

    const std::vector<subscription> &subscriptions() const
    {
        return m_subscriptions;
    }

  private:
    std::vector<subscription> m_subscriptions;

    event(const event &) = default;
    event &operator=(const event &) = default;

    friend class engine_events;
    friend class entity_events;
};
} // namespace ppx

#endif