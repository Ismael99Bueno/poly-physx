#ifndef EVENT_HPP
#define EVENT_HPP

#include <functional>
#include <vector>

namespace ppx
{
    template <class... Ts>
    class event final
    {
    public:
        event() { m_subscriptions.reserve(10); }

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

    private:
        std::vector<subscription> m_subscriptions;

        event(const event &) = delete;
        event &operator=(const event &) = delete;
    };
}

#endif