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

        void translate(const vec2 &dpos);
        void rotate(float angle);

        virtual const vec2 &pos() const;
        const vec2 &vel() const;
        const vec2 vel(const vec2 &at) const;
        virtual float angpos() const;
        float angvel() const;
        float mass() const;
        float charge() const;

        virtual void pos(const vec2 &pos);
        void vel(const vec2 &vel);
        virtual void angpos(float angpos);
        void angvel(float angvel);
        void mass(float mass);
        void charge(float charge);

    protected:
        vec2 m_pos, m_vel;
        float m_angvel, m_angpos;
        float m_mass, m_charge;
    };
}

#endif