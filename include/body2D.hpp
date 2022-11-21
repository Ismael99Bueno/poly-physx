#ifndef BODY2D_HPP
#define BODY2D_HPP

#include "vec2.hpp"

namespace phys
{
    class body2D
    {
    public:
        body2D(const alg::vec2 &pos = {0.f, 0.f},
               const alg::vec2 &vel = {0.f, 0.f},
               float angpos = 0.f, float angvel = 0.f,
               float mass = 1.f, float charge = 1.f);

        void translate(const alg::vec2 &dpos);
        void rotate(float angle);

        virtual const alg::vec2 &pos() const;
        const alg::vec2 &vel() const;
        const alg::vec2 vel(const alg::vec2 &at) const;
        virtual float angpos() const;
        float angvel() const;
        float mass() const;
        float charge() const;

        virtual void pos(const alg::vec2 &pos);
        void vel(const alg::vec2 &vel);
        virtual void angpos(float angpos);
        void angvel(float angvel);
        void mass(float mass);
        void charge(float charge);

    protected:
        alg::vec2 m_pos, m_vel;
        float m_angvel, m_angpos;
        float m_mass, m_charge;
    };
}

#endif