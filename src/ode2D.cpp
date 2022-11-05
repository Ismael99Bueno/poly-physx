#include "ode2D.hpp"

namespace physics
{
    std::vector<float> ode(float t, const std::vector<float> &state, engine2D &engine)
    {
        std::vector<float> result;
        result.reserve(state.size());

        engine.retrieve(state);
        engine.reset_accelerations(); // REWORK HOW ENTITIES SAVE FORCES!!
        for (std::size_t i = 0; i < engine.m_entities.size(); i++)
        {
            const entity2D &e = engine.m_entities[i];
            const std::pair<vec2, float> &accel = e.accel();

            const std::size_t j = VAR_PER_ENTITY * i;
            result.insert(result.end(), {state[j + 3], state[j + 4], state[j + 5],
                                         accel.first.x, accel.first.y, accel.second});
        }
        const std::vector<float> accels = engine.m_compeller.solve_constrains(state, result);
        for (std::size_t i = 0; i < engine.m_entities.size(); i++)
            for (std::size_t j = 0; j < 3; j++)
                result[6 * i + j + 3] += accels[3 * i + j];
        return result;
    }
}