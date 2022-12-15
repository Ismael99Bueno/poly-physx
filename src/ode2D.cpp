#include "ode2D.hpp"
#include "perf.hpp"

namespace phys
{
    std::vector<float> ode(float t, const std::vector<float> &state, engine2D &engine)
    {
        PERF_FUNCTION()
        DBG_ASSERT(state.size() == 6 * engine.size(), "State vector size must be exactly 6 times greater than the entity array size - state: %zu, entity array: %zu\n", state.size(), engine.size())
        std::vector<float> stchanges(state.size(), 0.f);

        engine.retrieve(state);
        engine.load_velocities_and_added_forces(stchanges);
        engine.load_interactions_and_externals(stchanges);
        const std::vector<float> inv_masses = engine.inverse_masses();

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