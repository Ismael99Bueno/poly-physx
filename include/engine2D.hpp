#ifndef ENGINE2D_HPP
#define ENGINE2D_HPP

#include "entity_ptr.hpp"

namespace physics
{
    class engine2D
    {
    public:
        engine2D(std::size_t allocations = 100);

        void retrieve();

        entity_ptr add(const vec2 &pos = {0.f, 0.f},
                       const vec2 &vel = {0.f, 0.f},
                       float angpos = 0.f, float angvel = 0.f,
                       float mass = 1.f, float charge = 1.f);

        const_entity_ptr get(std::size_t index) const;
        entity_ptr get(std::size_t index);

    private:
        std::vector<entity2D> m_entities;
        std::vector<float> m_state;
    };
}

#endif