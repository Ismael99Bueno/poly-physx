#ifndef BODY2D_HPP
#define BODY2D_HPP

#include "vec2.hpp"

namespace physics
{
    using namespace vec;
    class body2D
    {
    public:
        body2D(const vec2 &pos = {0.f, 0.f},
               const vec2 &vel = {0.f, 0.f},
               float angpos = 0.f, float angvel = 0.f,
               float mass = 1.f, float charge = 1.f);

        const vec2 &pos() const;
        const vec2 &vel() const;
        vec2 &pos();
        vec2 &vel();
        float mass() const;
        float charge() const;

        void pos(const vec2 &pos);
        void vel(const vec2 &vel);
        void mass(float mass);
        void charge(float charge);

    private:
        vec2 m_pos, m_vel;
        float m_angvel, m_angpos;
        float m_mass, m_charge;
    };
}

#endif