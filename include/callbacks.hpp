#ifndef CALLBACKS_HPP
#define CALLBACKS_HPP

#include "entity2D_ptr.hpp"
#include "constraint_interface2D.hpp"
#include "spring2D.hpp"
#include "pass_key.hpp"

namespace ppx
{
    class callbacks final
    {
    public:
        callbacks(engine_key);

        using add_entity = std::function<void(entity2D_ptr)>;
        using early_remove_entity = std::function<void(entity2D &)>;
        using late_remove_entity = std::function<void(std::size_t)>;

        using add_spring = std::function<void(spring2D *)>;
        using remove_spring = std::function<void(spring2D &)>;
        using constraint_cb = std::function<void(const std::shared_ptr<constraint_interface2D> &)>;

        void on_entity_addition(const add_entity &on_add);
        void on_early_entity_removal(const early_remove_entity &on_remove);
        void on_late_entity_removal(const late_remove_entity &on_remove);

        void on_spring_addition(const add_spring &on_add);
        void on_spring_removal(const remove_spring &on_remove);

        void on_constraint_addition(const constraint_cb &on_add);
        void on_constraint_removal(const constraint_cb &on_remove);

        void entity_addition(entity2D_ptr e) const;
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

        callbacks(const callbacks &) = delete;
        callbacks &operator=(const callbacks &) = delete;
    };

}

#endif