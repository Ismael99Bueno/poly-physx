#ifndef ENTITY_CALLBACKS_HPP
#define ENTITY_CALLBACKS_HPP

#include "pass_key.hpp"
#include <vector>
#include <functional>

namespace ppx
{
    struct collision2D;
    class entity2D_ptr;
    class entity_callbacks final
    {
    public:
        entity_callbacks(entity_key);

    private:
        using enter_stay_cb = std::function<void(const collision2D &)>;
        using exit_cb = std::function<void(const entity2D_ptr &)>;

    public:
        void on_collision_enter(const enter_stay_cb &on_enter);
        void on_collision_stay(const enter_stay_cb &on_stay);
        void on_collision_exit(const exit_cb &on_exit);

        void try_enter_or_stay(const collision2D &c) const;
        void try_exit(const entity2D_ptr &other) const;

    private:
        std::vector<enter_stay_cb> m_on_enter, m_on_stay;
        std::vector<exit_cb> m_on_exit;
        mutable bool m_in_collision = false;

        void on_enter(const collision2D &c) const;
        void on_stay(const collision2D &c) const;
        void on_exit(const entity2D_ptr &other) const;

        template <typename T, typename U>
        static void call(const std::vector<T> &cbs, const U &c)
        {
            for (const T &cb : cbs)
                cb(c);
        }

        entity_callbacks(const entity_callbacks &) = delete;
        entity_callbacks &operator=(const entity_callbacks &) = delete;
    };
}

#endif