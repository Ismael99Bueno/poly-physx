#ifndef ENTITY_CALLBACKS_HPP
#define ENTITY_CALLBACKS_HPP

#include "pass_key.hpp"
#include <vector>
#include <functional>
#include <unordered_set>

namespace ppx
{
    struct collision2D;
    class entity2D_ptr;
    class entity_callbacks final
    {
    public:
        entity_callbacks(entity_key, std::size_t allocations = 15);

    private:
        using enter_stay_cb = std::function<void(const collision2D &)>;
        using exit_cb = std::function<void(const entity2D_ptr &)>;

    public:
        void on_collision_enter(const enter_stay_cb &on_enter);
        void on_collision_stay(const enter_stay_cb &on_stay);
        void on_collision_exit(const exit_cb &on_exit);

        void try_enter_or_stay(const collision2D &c) const;
        void try_exit(const entity2D_ptr &incoming) const;
        void reset(engine_key);

    private:
        std::vector<enter_stay_cb> m_on_enter, m_on_stay;
        std::vector<exit_cb> m_on_exit;
        mutable bool m_processed = false;
        mutable std::unordered_set<std::size_t> m_collided_ids;

        void on_enter(const collision2D &c) const;
        void on_stay(const collision2D &c) const;
        void on_exit(const entity2D_ptr &incoming) const;

        template <typename T, typename U>
        static void call(const std::vector<T> &cbs, const U &c)
        {
            for (const T &cb : cbs)
                cb(c);
        }
    };
}

#endif