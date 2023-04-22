#include "ppx/spring2D.hpp"
#include <glm/geometric.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>

namespace ppx
{
    static float cross(const glm::vec2 &v1, const glm::vec2 &v2) { return v1.x * v2.y - v1.y * v2.x; }
    spring2D::spring2D(const entity2D_ptr &e1,
                       const entity2D_ptr &e2,
                       const float stiffness,
                       const float dampening,
                       const float length) : m_e1(e1),
                                             m_e2(e2),
                                             m_stiffness(stiffness),
                                             m_dampening(dampening),
                                             m_length(length),
                                             m_has_joints(false) {}

    spring2D::spring2D(const entity2D_ptr &e1,
                       const entity2D_ptr &e2,
                       const glm::vec2 &joint1,
                       const glm::vec2 &joint2,
                       const float stiffness,
                       const float dampening,
                       const float length) : m_e1(e1),
                                             m_e2(e2),
                                             m_joint1(joint1),
                                             m_joint2(joint2),
                                             m_stiffness(stiffness),
                                             m_dampening(dampening),
                                             m_angle1(e1->angpos()),
                                             m_angle2(e2->angpos()),
                                             m_length(length),
                                             m_has_joints(true) {}

    std::tuple<glm::vec2, float, float> spring2D::force() const
    {
        return m_has_joints ? with_joints_force() : without_joints_force();
    }

    std::tuple<glm::vec2, float, float> spring2D::without_joints_force() const
    {
        const glm::vec2 relpos = m_e2->pos() - m_e1->pos(),
                        direction = glm::normalize(relpos),
                        relvel = direction * glm::dot(m_e2->vel() - m_e1->vel(), direction),
                        vlen = m_length * direction;
        return {m_stiffness * (relpos - vlen) + m_dampening * relvel, 0.f, 0.f};
    }

    std::tuple<glm::vec2, float, float> spring2D::with_joints_force() const
    {
        const glm::vec2 rot_joint1 = joint1(),
                        rot_joint2 = joint2();
        const glm::vec2 p1 = m_e1->pos() + rot_joint1,
                        p2 = m_e2->pos() + rot_joint2;
        const glm::vec2 relpos = p2 - p1,
                        direction = glm::normalize(relpos),
                        relvel = direction * glm::dot(m_e2->vel_at(rot_joint2) - m_e1->vel_at(rot_joint1), direction),
                        vlen = m_length * direction;

        const glm::vec2 force = m_stiffness * (relpos - vlen) + m_dampening * relvel;
        const float torque1 = cross(rot_joint1, force), torque2 = cross(force, rot_joint2);
        return {force, torque1, torque2};
    }

    bool spring2D::try_validate() { return m_e1.try_validate() && m_e2.try_validate(); }

    float spring2D::stiffness() const { return m_stiffness; }
    float spring2D::dampening() const { return m_dampening; }
    float spring2D::length() const { return m_length; }

    void spring2D::stiffness(const float stiffness) { m_stiffness = stiffness; }
    void spring2D::dampening(const float dampening) { m_dampening = dampening; }
    void spring2D::length(const float length) { m_length = length; }

    void spring2D::write(ini::output &out) const
    {
        out.write("e1", m_e1->index());
        out.write("e2", m_e2->index());
        const glm::vec2 j1 = joint1(), j2 = joint2();
        out.write("joint1x", j1.x);
        out.write("joint1y", j1.y);
        out.write("joint2x", j2.x);
        out.write("joint2y", j2.y);
        out.write("stiffness", m_stiffness);
        out.write("dampening", m_dampening);
        out.write("length", m_length);
        out.write("has_joints", m_has_joints);
    }

    void spring2D::read(ini::input &in)
    {
        m_stiffness = in.readf32("stiffness");
        m_dampening = in.readf32("dampening");
        m_length = in.readf32("length");
    }

    float spring2D::kinetic_energy() const { return m_e1->kinetic_energy() + m_e2->kinetic_energy(); }
    float spring2D::potential_energy() const
    {
        const glm::vec2 p1 = m_e1->pos() + joint1(),
                        p2 = m_e2->pos() + joint2();
        return 0.5f * m_stiffness * glm::distance2(p1, p2);
    }
    float spring2D::energy() const { return kinetic_energy() + potential_energy(); }

    const entity2D_ptr &spring2D::e1() const { return m_e1; }
    const entity2D_ptr &spring2D::e2() const { return m_e2; }

    glm::vec2 spring2D::joint1() const { return glm::rotate(m_joint1, m_e1->angpos() - m_angle1); }
    glm::vec2 spring2D::joint2() const { return glm::rotate(m_joint2, m_e2->angpos() - m_angle2); }

    void spring2D::joint1(const glm::vec2 &joint1)
    {
        m_joint1 = joint1;
        m_angle1 = m_e1->angpos();
        m_has_joints = true;
    }
    void spring2D::joint2(const glm::vec2 &joint2)
    {
        m_joint2 = joint2;
        m_angle2 = m_e2->angpos();
        m_has_joints = true;
    }

    bool spring2D::has_joints() const { return m_has_joints; }
}