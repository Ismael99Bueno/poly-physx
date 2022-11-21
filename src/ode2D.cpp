#include "ode2D.hpp"

#define POS_PER_ENTITY 3

namespace phys
{
    std::vector<float> ode(float t, const std::vector<float> &state, engine2D &engine)
    {
        std::vector<float> stchanges, inv_masses;
        stchanges.reserve(state.size());
        inv_masses.reserve(state.size() / 2);

        engine.retrieve(state);
        for (std::size_t i = 0; i < engine.m_entities.size(); i++)
        {
            const entity2D &e = engine.m_entities[i];
            const std::pair<alg::vec2, float> &force = e.force();

            const std::size_t j = VAR_PER_ENTITY * i;
            stchanges.insert(stchanges.end(), {state[j + 3], state[j + 4], state[j + 5],
                                               force.first.x, force.first.y, force.second});
            inv_masses.insert(inv_masses.end(), {1.0f / e.mass(), 1.0f / e.mass(), 1.0f / e.inertia()});
        }
        engine.m_collider.solve_and_load_collisions(stchanges);
        engine.m_compeller.solve_and_load_constrains(stchanges, inv_masses);
        for (std::size_t i = 0; i < engine.m_entities.size(); i++)
        {
            stchanges[VAR_PER_ENTITY * i + 3] *= inv_masses[POS_PER_ENTITY * i];
            stchanges[VAR_PER_ENTITY * i + 4] *= inv_masses[POS_PER_ENTITY * i + 1];
            stchanges[VAR_PER_ENTITY * i + 5] *= inv_masses[POS_PER_ENTITY * i + 2];
        }
        return stchanges;
    }
}