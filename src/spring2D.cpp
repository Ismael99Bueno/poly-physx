#include "spring2D.hpp"

namespace phys
{
    spring2D::spring2D(const const_entity_ptr &e1,
                       const const_entity_ptr &e2,
                       const float length) : m_e1(e1),
                                             m_e2(e2),
                                             m_joint1(e1->pos()),
                                             m_joint2(e2->pos()),
                                             m_length(length),
                                             m_has_joints(false) {}

    spring2D::spring2D(const const_entity_ptr &e1,
                       const const_entity_ptr &e2,
                       const alg::vec2 &joint1,
                       const alg::vec2 &joint2,
                       const float length) : m_e1(e1),
                                             m_e2(e2),
                                             m_joint1(joint1),
                                             m_joint2(joint2),
                                             m_length(length),
                                             m_has_joints(true) {}

    std::tuple<alg::vec2, float, float> spring2D::force() const
    {
        return m_has_joints ? with_joints_force() : without_joints_force();
    }

    std::tuple<alg::vec2, float, float> spring2D::without_joints_force() const
    {
        const alg::vec2 relpos = m_e2->pos() - m_e1->pos(),
                        direction = relpos.normalized(),
                        relvel = direction * (m_e2->vel() - m_e1->vel()).dot(direction),
                        vlen = m_length * direction;
        return {m_stiffness * (relpos - vlen) + m_dampening * relvel, 0.f, 0.f};
    }

    std::tuple<alg::vec2, float, float> spring2D::with_joints_force() const
    {
        const alg::vec2 rot_joint1 = m_joint1.rotated(m_e1->angpos()),
                        rot_joint2 = m_joint2.rotated(m_e2->angpos());
        const alg::vec2 p1 = m_e1->pos() + rot_joint1,
                        p2 = m_e2->pos() + rot_joint2;
        const alg::vec2 relpos = p2 - p1,
                        direction = relpos.normalized(),
                        relvel = direction * (m_e2->vel(rot_joint2) - m_e1->vel(rot_joint1)).dot(direction),
                        vlen = m_length * direction;
        const alg::vec2 force = m_stiffness * (relpos - vlen) + m_dampening * relvel;
        const float torque1 = rot_joint1.cross(force), torque2 = force.cross(rot_joint2);
        return {force, torque1, torque2};
    }

    bool spring2D::try_validate() { return m_e1.try_validate() && m_e2.try_validate(); }

    float spring2D::stiffness() const { return m_stiffness; }
    float spring2D::dampening() const { return m_dampening; }
    float spring2D::length() const { return m_length; }

    void spring2D::stiffness(const float stiffness) { m_stiffness = stiffness; }
    void spring2D::dampening(const float dampening) { m_dampening = dampening; }
    void spring2D::length(const float length) { m_length = length; }

    const_entity_ptr spring2D::e1() const { return m_e1; }
    const_entity_ptr spring2D::e2() const { return m_e2; }

    const alg::vec2 &spring2D::joint1() const { return m_joint1; }
    const alg::vec2 &spring2D::joint2() const { return m_joint2; }

    void spring2D::joint1(const alg::vec2 &joint1)
    {
        m_joint1 = joint1;
        m_has_joints = true;
    }
    void spring2D::joint2(const alg::vec2 &joint2)
    {
        m_joint2 = joint2;
        m_has_joints = true;
    }
}