#include "ode2D.hpp"

namespace physics
{
    std::vector<float> ode(float t, const std::vector<float> &state, engine2D &engine)
    {
        std::vector<float> stchanges;
        stchanges.reserve(state.size());

        engine.retrieve(state);
        engine.reset_accelerations(); // REWORK HOW ENTITIES SAVE FORCES!!
        for (std::size_t i = 0; i < engine.m_entities.size(); i++)
        {
            const entity2D &e = engine.m_entities[i];
            const std::pair<vec2, float> &accel = e.accel();

            const std::size_t j = VAR_PER_ENTITY * i;
            stchanges.insert(stchanges.end(), {state[j + 3], state[j + 4], state[j + 5],
                                               accel.first.x, accel.first.y, accel.second});
        }
        engine.m_collider.solve_and_load_collisions(stchanges);
        engine.m_compeller.solve_and_load_constrains(stchanges);
        return stchanges;
    }
}