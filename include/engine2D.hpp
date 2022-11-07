#ifndef ENGINE2D_HPP
#define ENGINE2D_HPP

#include "integrator.hpp"
#include "entity_ptr.hpp"
#include "compeller2D.hpp"

namespace physics
{
    class engine2D
    {
    public:
        engine2D(const rk::tableau &table, float dt = 0.001f, std::size_t allocations = 100);

        void retrieve();

        bool raw_forward();
        bool reiterative_forward(std::size_t reiterations = 2);
        bool embedded_forward();

        virtual entity_ptr add(const vec2 &pos = {0.f, 0.f},
                               const vec2 &vel = {0.f, 0.f},
                               float angpos = 0.f, float angvel = 0.f,
                               float mass = 1.f, float charge = 1.f);
        void add(const constrain_interface &c);

        const_entity_ptr get(std::size_t index) const;
        entity_ptr get(std::size_t index);

        rk::integrator &integrator();
        float elapsed() const;

    protected:
        std::vector<entity2D> m_entities;

    private:
        std::vector<float> m_state;
        compeller2D m_compeller;

        rk::integrator m_integ;
        float m_t = 0.f, m_dt = 0.001f;

        void retrieve(const std::vector<float> &state);
        void reset_accelerations();

        friend std::vector<float> ode(float t, const std::vector<float> &state, engine2D &engine);
    };
}

#endif