#include "ode2D.hpp"

namespace phys
{
    std::vector<float> ode(float t, const std::vector<float> &state, engine2D &engine)
    {
        std::vector<float> stchanges(state.size(), 0.f);

        engine.retrieve(state);
        engine.load_velocities_and_added_forces(stchanges);
        engine.load_interactions_and_externals(stchanges);
        const std::vector<float> inv_masses = engine.inverse_masses();

        engine.m_collider.solve_and_load_collisions(stchanges);
        engine.m_compeller.solve_and_load_constrains(stchanges, inv_masses);
        for (std::size_t i = 0; i < engine.m_entities.size(); i++)
        {
            stchanges[6 * i + 3] *= inv_masses[3 * i];
            stchanges[6 * i + 4] *= inv_masses[3 * i + 1];
            stchanges[6 * i + 5] *= inv_masses[3 * i + 2];
        }
        return stchanges;
    }
}