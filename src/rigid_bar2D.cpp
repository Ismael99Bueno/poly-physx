#include "rigid_bar2D.hpp"

namespace phys
{
    rigid_bar2D::rigid_bar2D(const entity_ptr &e1,
                             const entity_ptr &e2,
                             const float length) : constraint2D<2>({e1, e2}),
                                                   m_length(length),
                                                   m_has_joints(false) {}

    rigid_bar2D::rigid_bar2D(const entity_ptr &e1,
                             const entity_ptr &e2,
                             const alg::vec2 &joint1,
                             const alg::vec2 &joint2,
                             const float length) : constraint2D<2>({e1, e2}),
                                                   m_joint1(joint1),
                                                   m_joint2(joint2),
                                                   m_angle1(e1->angpos()),
                                                   m_angle2(e2->angpos()),
                                                   m_length(length),
                                                   m_has_joints(true) {}

    float rigid_bar2D::constraint(const std::array<const_entity_ptr, 2> &entities) const
    {
        return m_has_joints ? with_joints_constraint(entities) : without_joints_constraint(entities);
    }

    float rigid_bar2D::constraint_derivative(const std::array<const_entity_ptr, 2> &entities) const
    {
        return m_has_joints ? with_joints_constraint_derivative(entities) : without_joints_constraint_derivative(entities);
    }

    float rigid_bar2D::without_joints_constraint(const std::array<const_entity_ptr, 2> &entities) const
    {
        const phys::const_entity_ptr &e1 = entities[0], &e2 = entities[1];
        return e1->pos().sq_dist(e2->pos()) - m_length * m_length;
    }
    float rigid_bar2D::without_joints_constraint_derivative(const std::array<const_entity_ptr, 2> &entities) const
    {
        const phys::const_entity_ptr &e1 = entities[0], &e2 = entities[1];
        return 2.f * (e1->pos() - e2->pos())
                         .dot(e1->vel() - e2->vel());
    }

    float rigid_bar2D::with_joints_constraint(const std::array<const_entity_ptr, 2> &entities) const
    {
        const phys::const_entity_ptr &e1 = entities[0], &e2 = entities[1];
        const alg::vec2 p1 = m_joint1.rotated(e1->angpos() - m_angle1) + e1->pos(),
                        p2 = m_joint2.rotated(e2->angpos() - m_angle2) + e2->pos();

        return p1.sq_dist(p2) - m_length * m_length;
    }
    float rigid_bar2D::with_joints_constraint_derivative(const std::array<const_entity_ptr, 2> &entities) const
    {
        const phys::const_entity_ptr &e1 = entities[0], &e2 = entities[1];
        const alg::vec2 rot_joint1 = m_joint1.rotated(e1->angpos()),
                        rot_joint2 = m_joint2.rotated(e2->angpos());

        return 2.f * (rot_joint1 - rot_joint2 + e1->pos() - e2->pos())
                         .dot(e1->vel_at(rot_joint1) - e2->vel_at(rot_joint2));
    }

    float rigid_bar2D::length() const { return m_length; }
    void rigid_bar2D::length(const float length) { m_length = length; }

    const_entity_ptr rigid_bar2D::e1() const { return m_entities[0]; }
    const_entity_ptr rigid_bar2D::e2() const { return m_entities[1]; }

    alg::vec2 rigid_bar2D::joint1() const { return m_joint1.rotated(m_entities[0]->angpos() - m_angle1); }
    alg::vec2 rigid_bar2D::joint2() const { return m_joint2.rotated(m_entities[1]->angpos() - m_angle2); }

    void rigid_bar2D::joint1(const alg::vec2 &joint1)
    {
        m_joint1 = joint1;
        m_angle1 = m_entities[0]->angpos();
        m_has_joints = true;
    }
    void rigid_bar2D::joint2(const alg::vec2 &joint2)
    {
        m_joint2 = joint2;
        m_angle2 = m_entities[1]->angpos();
        m_has_joints = true;
    }
}