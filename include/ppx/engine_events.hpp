#ifndef ENGINE_EVENTS_HPP
#define ENGINE_EVENTS_HPP

#include "ppx/entity2D_ptr.hpp"
#include "ppx/constraint_interface2D.hpp"
#include "ppx/spring2D.hpp"
#include "ppx/pass_key.hpp"
#include <memory>

namespace ppx
{
    class engine_events final
    {
    public:
        engine_events(engine_key);

    private:
        using add_entity = std::function<void(const entity2D_ptr &)>;
        using early_remove_entity = std::function<void(entity2D &)>;
        using late_remove_entity = std::function<void(std::size_t)>;

        using add_spring = std::function<void(spring2D *)>;
        using remove_spring = std::function<void(spring2D &)>;
        using constraint_cb = std::function<void(const std::shared_ptr<constraint_interface2D> &)>;

    public:
        void on_entity_addition(const add_entity &on_add);
        void on_early_entity_removal(const early_remove_entity &on_remove);
        void on_late_entity_removal(const late_remove_entity &on_remove);

        void on_spring_addition(const add_spring &on_add);
        void on_spring_removal(const remove_spring &on_remove);

        void on_constraint_addition(const constraint_cb &on_add);
        void on_constraint_removal(const constraint_cb &on_remove);

        void entity_addition(const entity2D_ptr &e) const;
        void early_entity_removal(entity2D &e) const;
        void late_entity_removal(std::size_t index) const;

        void spring_addition(spring2D *sp) const;
        void spring_removal(spring2D &sp) const;

        void constraint_addition(const std::shared_ptr<constraint_interface2D> &) const;
        void constraint_removal(const std::shared_ptr<constraint_interface2D> &) const;

    private:
        std::vector<add_entity> m_on_entity_addition;
        std::vector<early_remove_entity> m_on_early_entity_removal;
        std::vector<late_remove_entity> m_on_late_entity_removal;
        std::vector<add_spring> m_on_spring_addition;
        std::vector<remove_spring> m_on_spring_removal;
        std::vector<constraint_cb> m_on_constraint_addition, m_on_constraint_removal;

        engine_events(const engine_events &) = delete;
        engine_events &operator=(const engine_events &) = delete;
    };

}

#endif