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

        using entity_cb = std::function<void(entity2D_ptr)>;
        using late_remove_entity_cb = std::function<void(std::size_t)>;

        using spring_cb = std::function<void(spring2D *)>;
        using constraint_cb = std::function<void(std::shared_ptr<constraint_interface2D>)>;

        void on_entity_addition(const entity_cb &on_add);
        void on_early_entity_removal(const entity_cb &on_remove);
        void on_late_entity_removal(const late_remove_entity_cb &on_remove);

        void on_spring_addition(const spring_cb &on_add);
        void on_spring_removal(const spring_cb &on_add);

        void on_constraint_addition(const constraint_cb &on_add);
        void on_constraint_removal(const constraint_cb &on_add);

        void entity_addition(engine_key) const;
        void early_entity_removal(engine_key) const;
        void late_entity_removal(engine_key) const;

        void spring_addition(engine_key) const;
        void spring_removal(engine_key) const;

        void constraint_addition(engine_key) const;
        void constraint_removal(engine_key) const;

    private:
        std::vector<entity_cb> m_on_entity_addition, m_on_early_entity_removal;
        std::vector<late_remove_entity_cb> m_on_late_entity_removal;
        std::vector<spring_cb> m_on_spring_addition, m_on_spring_removal;
        std::vector<constraint_cb> m_on_constraint_addition, m_on_constraint_removal;

        callbacks(const callbacks &) = delete;
        callbacks &operator=(const callbacks &) = delete;
    };

}

#endif