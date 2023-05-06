#ifndef ENGINE2D_HPP
#define ENGINE2D_HPP

#include "rk/integrator.hpp"
#include "ppx/entity2D_ptr.hpp"
#include "ppx/compeller2D.hpp"
#include "ppx/collider2D.hpp"
#include "ppx/force2D.hpp"
#include "ppx/interaction2D.hpp"
#include "ppx/spring2D.hpp"
#include "cvw/container_view.hpp"
#include "ppx/rigid_bar2D.hpp"
#include "ppx/engine_events.hpp"
#include "rk/tableaus.hpp"

namespace ppx
{
    class engine2D
    {
    public:
        engine2D(const rk::butcher_tableau &table = rk::rk4,
                 std::size_t allocations = 100);

        void retrieve();

        bool raw_forward(float &timestep);
        bool reiterative_forward(float &timestep, std::uint8_t reiterations = 2);
        bool embedded_forward(float &timestep);

        template <class... Args>
        entity2D_ptr add_entity(Args &&...args)
        {
            entity2D &e = m_entities.emplace_back(std::forward<Args>(args)...);
            return process_entity_addition(e);
        }

        bool remove_entity(std::size_t index);
        bool remove_entity(const entity2D &e);

        template <typename T, class... Args>
        std::shared_ptr<T> add_force(Args &&...args)
        {
            static_assert(std::is_convertible<T *, force2D *>::value, "Force must inherit from force2D!");
            const auto force = std::make_shared<T>(std::forward<Args>(args)...);
            m_forces.push_back(force);
            m_events.on_force_addition(force);
            return force;
        }
        template <typename T, class... Args>
        std::shared_ptr<T> add_interaction(Args &&...args)
        {
            static_assert(std::is_convertible<T *, interaction2D *>::value, "Force must inherit from interaction2D!");
            const auto inter = std::make_shared<T>(std::forward<Args>(args)...);
            m_interactions.push_back(inter);
            m_events.on_interaction_addition(inter);
            return inter;
        }

        template <class... Args>
        spring2D &add_spring(Args &&...args)
        {
            spring2D &sp = m_springs.emplace_back(std::forward<Args>(args)...);
            m_events.on_spring_addition(&sp);
            return sp;
        }

        template <typename T, class... Args>
        std::shared_ptr<T> add_constraint(Args &&...args) { return m_compeller.add_constraint<T>(std::forward<Args>(args)...); }

        bool remove_constraint(const std::shared_ptr<const constraint_interface2D> &ctr);

        bool remove_force(const std::shared_ptr<const force2D> &force);
        bool remove_interaction(const std::shared_ptr<const interaction2D> &inter);
        bool remove_spring(std::size_t index);
        bool remove_spring(const spring2D &sp);

        void clear_entities();
        void clear_forces();
        void clear_interactions();
        void clear_springs();
        void clear_constraints();
        void clear();

        void checkpoint();
        void revert();

        float kinetic_energy() const;
        float potential_energy() const;
        float energy() const;

        const_entity2D_ptr entity_from_index(std::size_t index) const;
        entity2D_ptr entity_from_index(std::size_t index);

        const_entity2D_ptr entity_from_id(std::size_t id) const;
        entity2D_ptr entity_from_id(std::size_t id);

        const spring2D *spring_from_entities(const entity2D &e1, const entity2D &e2) const;
        spring2D *spring_from_entities(const entity2D &e1, const entity2D &e2);

        std::shared_ptr<const rigid_bar2D> rbar_from_entities(const entity2D &e1, const entity2D &e2) const;
        std::shared_ptr<rigid_bar2D> rbar_from_entities(const entity2D &e1, const entity2D &e2);

        const_entity2D_ptr operator[](std::size_t index) const;
        entity2D_ptr operator[](std::size_t index);

        std::vector<const_entity2D_ptr> operator[](const geo::aabb2D &aabb) const;
        std::vector<entity2D_ptr> operator[](const geo::aabb2D &aabb);

        const_entity2D_ptr operator[](const glm::vec2 &point) const;
        entity2D_ptr operator[](const glm::vec2 &point);

        const std::vector<std::shared_ptr<force2D>> &forces() const;
        const std::vector<std::shared_ptr<interaction2D>> &interactions() const;
        const std::vector<spring2D> &springs() const;

        cvw::vector<std::shared_ptr<force2D>> forces();
        cvw::vector<std::shared_ptr<interaction2D>> interactions();
        cvw::vector<spring2D> springs();

        const std::vector<entity2D> &entities() const;
        cvw::vector<entity2D> entities();
        std::size_t size() const;

        const rk::integrator &integrator() const;
        rk::integrator &integrator();

        const collider2D &collider() const;
        collider2D &collider();

        const compeller2D &compeller() const;
        engine_events &events();

        float elapsed() const;

    private:
        std::vector<entity2D> m_entities;
        collider2D m_collider;
        compeller2D m_compeller;
        std::vector<std::shared_ptr<force2D>> m_forces;
        std::vector<std::shared_ptr<interaction2D>> m_interactions;
        std::vector<spring2D> m_springs;
        std::tuple<float, std::vector<float>, std::vector<entity2D>> m_checkpoint;
        rk::integrator m_integ;
        engine_events m_events;

        float m_elapsed = 0.f;

        entity2D_ptr process_entity_addition(entity2D &e);

        void load_velocities_and_added_forces(std::vector<float> &stchanges) const;
        void load_interactions_and_externals(std::vector<float> &stchanges) const;
        std::vector<float> inverse_masses() const;
        void reset_entities();
        void retrieve(const std::vector<float> &vars_buffer);
        void register_forces_into_entities();
        void validate();

        static void load_force(std::vector<float> &stchanges,
                               const glm::vec2 &force,
                               float torque,
                               std::size_t index);

        friend std::vector<float> ode(float t, const std::vector<float> &state, engine2D &engine);
        engine2D(const engine2D &) = delete;
        engine2D &operator=(const engine2D &) = delete;

#ifdef HAS_YAML_CPP
        friend YAML::Emitter &operator<<(YAML::Emitter &, const engine2D &);
        friend struct YAML::convert<engine2D>;
#endif
    };
#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const engine2D &eng);
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    template <>
    struct convert<ppx::engine2D>
    {
        static Node encode(const ppx::engine2D &eng);
        static bool decode(const Node &node, ppx::engine2D &eng);
    };
}
#endif

#endif