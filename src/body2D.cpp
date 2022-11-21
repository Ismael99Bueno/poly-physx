#include "body2D.hpp"

namespace phys
{
    body2D::body2D(const alg::vec2 &pos,
                   const alg::vec2 &vel,
                   const float angpos,
                   const float angvel,
                   const float mass,
                   const float charge) : m_pos(pos), m_vel(vel),
                                         m_angpos(angpos), m_angvel(angvel),
                                         m_mass(mass), m_charge(charge) {}

    void body2D::translate(const alg::vec2 &dpos) { pos(m_pos + dpos); }
    void body2D::rotate(float angle) { angpos(m_angpos + angle); }

    const alg::vec2 &body2D::pos() const { return m_pos; }
    const alg::vec2 &body2D::vel() const { return m_vel; }
    const alg::vec2 body2D::vel(const alg::vec2 &at) const { return m_vel + m_angvel * alg::vec2(m_pos.y - at.y, at.x - m_pos.x); }

    float body2D::angpos() const { return m_angpos; }
    float body2D::angvel() const { return m_angvel; }

    float body2D::mass() const { return m_mass; }
    float body2D::charge() const { return m_charge; }

    void body2D::pos(const alg::vec2 &pos) { m_pos = pos; }
    void body2D::vel(const alg::vec2 &vel) { m_vel = vel; }

    void body2D::angpos(const float angpos) { m_angpos = angpos; }
    void body2D::angvel(const float angvel) { m_angvel = angvel; }

    void body2D::mass(const float mass) { m_mass = mass; }
    void body2D::charge(const float charge) { m_charge = charge; }
}