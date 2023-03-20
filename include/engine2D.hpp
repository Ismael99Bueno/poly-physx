#ifndef ENGINE2D_HPP
#define ENGINE2D_HPP

#include "integrator.hpp"
#include "entity2D_ptr.hpp"
#include "compeller2D.hpp"
#include "collider2D.hpp"
#include "force2D.hpp"
#include "interaction2D.hpp"
#include "spring2D.hpp"
#include "vector_view.hpp"

namespace phys
{
    class engine2D : public ini::saveable
    {
    public:
        engine2D(const rk::butcher_tableau &table, std::size_t allocations = 100);
        engine2D(const engine2D &eng);
        engine2D &operator=(const engine2D &eng) = delete;

        void retrieve();

        bool raw_forward(float &timestep);
        bool reiterative_forward(float &timestep, std::size_t reiterations = 2);
        bool embedded_forward(float &timestep);

        entity2D_ptr add_entity(const alg::vec2 &pos = alg::vec2::zero,
                                const alg::vec2 &vel = alg::vec2::zero,
                                float angpos = 0.f, float angvel = 0.f,
                                float mass = 1.f, float charge = 1.f,
                                const std::vector<alg::vec2> &vertices = geo::polygon2D::box(1.f),
                                bool kinematic = true);

        void remove_entity(std::size_t index);
        void remove_entity(const entity2D &e);

        void add_force(const std::shared_ptr<force2D> &force);
        void add_interaction(const std::shared_ptr<interaction2D> &inter);
        void add_spring(const spring2D &spring);

        void remove_force(const std::shared_ptr<force2D> &force);
        void remove_interaction(const std::shared_ptr<interaction2D> &inter);
        void remove_spring(std::size_t index);

        void clear_entities();
        void clear_forces();
        void clear_interactions();
        void clear_springs();
        void clear();

        void write(ini::output &out) const override;
        void read(ini::input &in) override;

        void checkpoint();
        void revert();

        float kinetic_energy() const;
        float potential_energy() const;
        float energy() const;

        using add_callback = std::function<void(entity2D_ptr)>;
        using remove_callback = std::function<void(std::size_t)>;
        void on_entity_addition(const add_callback &on_add);
        void on_entity_removal(const remove_callback &on_remove);

        const_entity2D_ptr operator[](std::size_t index) const;
        entity2D_ptr operator[](std::size_t index);

        std::vector<const_entity2D_ptr> operator[](const geo::aabb2D &aabb) const;
        std::vector<entity2D_ptr> operator[](const geo::aabb2D &aabb);

        const_entity2D_ptr operator[](const alg::vec2 &point) const;
        entity2D_ptr operator[](const alg::vec2 &point);

        const std::vector<std::shared_ptr<force2D>> &forces() const;
        const std::vector<std::shared_ptr<interaction2D>> &interactions() const;
        const std::vector<spring2D> &springs() const;

        utils::vector_view<std::shared_ptr<force2D>> forces();
        utils::vector_view<std::shared_ptr<interaction2D>> interactions();
        utils::vector_view<spring2D> springs();

        const std::vector<entity2D> &entities() const;
        utils::vector_view<entity2D> entities();
        std::size_t size() const;

        const rk::integrator &integrator() const;
        rk::integrator &integrator();

        const compeller2D &compeller() const;
        compeller2D &compeller();

        const collider2D &collider() const;
        collider2D &collider();

        float elapsed() const;

    private:
        std::vector<entity2D> m_entities;
        collider2D m_collider;
        compeller2D m_compeller;
        std::vector<std::shared_ptr<force2D>> m_forces;
        std::vector<std::shared_ptr<interaction2D>> m_interactions;
        std::vector<spring2D> m_springs;
        std::vector<add_callback> m_on_entity_addition;
        std::vector<remove_callback> m_on_entity_removal;
        std::tuple<float, std::vector<float>, std::vector<entity2D>> m_checkpoint;

        rk::integrator m_integ;
        float m_elapsed = 0.f;

        void load_velocities_and_added_forces(std::vector<float> &stchanges) const;
        void load_interactions_and_externals(std::vector<float> &stchanges) const;
        std::vector<float> inverse_masses() const;
        void reset_forces();
        void retrieve(const std::vector<float> &vars_buffer);
        void register_forces_into_entities();
        void validate();

        static void load_force(std::vector<float> &stchanges,
                               const alg::vec2 &force,
                               float torque,
                               std::size_t index);

        friend std::vector<float> ode(float t, const std::vector<float> &state, engine2D &engine);
    };
}

#endif