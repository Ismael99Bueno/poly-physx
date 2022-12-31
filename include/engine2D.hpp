#ifndef ENGINE2D_HPP
#define ENGINE2D_HPP

#include "integrator.hpp"
#include "entity_ptr.hpp"
#include "compeller2D.hpp"
#include "collider2D.hpp"
#include "force2D.hpp"
#include "interaction2D.hpp"
#include "spring2D.hpp"

namespace phys
{
    class engine2D
    {
    public:
        engine2D(const rk::butcher_tableau &table, std::size_t allocations = 100);

        void retrieve();

        bool raw_forward(float &dt);
        bool reiterative_forward(float &dt, std::size_t reiterations = 2);
        bool embedded_forward(float &dt);

        entity_ptr add_entity(const alg::vec2 &pos = {0.f, 0.f},
                              const alg::vec2 &vel = {0.f, 0.f},
                              float angpos = 0.f, float angvel = 0.f,
                              float mass = 1.f, float charge = 1.f,
                              const std::vector<alg::vec2> &vertices = geo::polygon2D::box(1.f),
                              bool dynamic = true);

        void remove_entity(std::size_t index);
        void remove_entity(const const_entity_ptr &e);

        void add_force(force2D *force);
        void add_interaction(interaction2D *inter);
        void add_spring(const spring2D &spring);

        void remove_force(force2D *force);
        void remove_interaction(interaction2D *inter);
        void remove_spring(std::size_t index);

        void clear_entities();
        void clear_forces();
        void clear_interactions();
        void clear_springs();
        void clear();

        using add_callback = std::function<void(entity_ptr)>;
        using remove_callback = std::function<void(std::size_t)>;
        void on_entity_addition(const add_callback &on_add);
        void on_entity_removal(const remove_callback &on_remove);

        const_entity_ptr operator[](std::size_t index) const;
        entity_ptr operator[](std::size_t index);

        std::unordered_set<const_entity_ptr> operator[](const geo::aabb2D &aabb) const;
        std::unordered_set<entity_ptr> operator[](const geo::aabb2D &aabb);

        const_entity_ptr operator[](const alg::vec2 &point) const;
        entity_ptr operator[](const alg::vec2 &point);

        const std::vector<entity2D> &entities() const;
        std::vector<entity2D> &entities();
        std::size_t size() const;

        const rk::integrator &integrator() const;
        rk::integrator &integrator();

        const std::vector<float> &state() const;

        const compeller2D &compeller() const;
        compeller2D &compeller();

        const collider2D &collider() const;
        collider2D &collider();

        float elapsed() const;

    private:
        std::vector<entity2D> m_entities;
        std::vector<float> m_state;
        compeller2D m_compeller;
        collider2D m_collider;
        std::unordered_set<force2D *> m_forces;
        std::unordered_set<interaction2D *> m_inters;
        std::vector<spring2D> m_springs;
        std::vector<add_callback> m_on_entity_addition;
        std::vector<remove_callback> m_on_entity_removal;

        rk::integrator m_integ;
        float m_t = 0.f;

        void load_velocities_and_added_forces(std::vector<float> &stchanges) const;
        void load_interactions_and_externals(std::vector<float> &stchanges) const;
        std::vector<float> inverse_masses() const;
        void reset_forces();
        void retrieve(const std::vector<float> &state);
        void register_forces_onto_entities();

        static void load_force(std::vector<float> &stchanges,
                               const alg::vec2 &force,
                               float torque,
                               std::size_t index);

        friend std::vector<float> ode(float t, const std::vector<float> &state, engine2D &engine);
    };
}

#endif