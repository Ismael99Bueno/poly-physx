#include "ode2D.hpp"

namespace physics
{
    std::vector<float> ode(float t, const std::vector<float> &state, engine2D &engine)
    {
        std::vector<float> result;
        result.reserve(state.size());

        engine.retrieve(state);
        for (std::size_t i = 0; i < engine.m_entities.size(); i++)
        {
            entity2D &e = engine.m_entities[i];
            const std::pair<vec2, float> &accel = e.compute_accel();
            result.insert(result.end(), {state[i + 3], state[i + 4], state[i + 5],
                                         accel.first.x, accel.first.y, accel.second});
        }
        return result;
    }
}