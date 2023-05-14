#include "ppx/pch.hpp"
#include "ppx/ode2D.hpp"
#include "perf/perf.hpp"

namespace ppx
{
    std::vector<float> ode(float t, const std::vector<float> &vars, engine2D &engine)
    {
        PERF_FUNCTION()
        DBG_ASSERT_CRITICAL(vars.size() == 6 * engine.size(), "State vector size must be exactly 6 times greater than the entity array size - vars: {0}, entity array: {1}", vars.size(), engine.size())
        std::vector<float> stchanges(vars.size(), 0.f);

        engine.retrieve(vars);
        engine.load_velocities_and_added_forces(stchanges);
        engine.load_interactions_and_externals(stchanges);
        const std::vector<float, mem::stack_allocator<float>> inv_masses = engine.effective_inverse_masses<mem::stack_allocator<float>>();

        engine.m_collider.solve_and_load_collisions(stchanges);
        engine.m_compeller.solve_and_load_constraints(stchanges, inv_masses);
        for (std::size_t i = 0; i < engine.size(); i++)
        {
            stchanges[6 * i + 3] *= inv_masses[3 * i];
            stchanges[6 * i + 4] *= inv_masses[3 * i + 1];
            stchanges[6 * i + 5] *= inv_masses[3 * i + 2];
        }
        return stchanges;
    }
}