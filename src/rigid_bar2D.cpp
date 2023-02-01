#include "rigid_bar2D.hpp"
#include "debug.hpp"
#include <cmath>

namespace phys
{
    rigid_bar2D::rigid_bar2D(const entity2D_ptr &e1,
                             const entity2D_ptr &e2,
                             const float length) : constraint2D<2>({e1, e2}),
                                                   m_length(length),
                                                   m_has_joints(false) {}

    rigid_bar2D::rigid_bar2D(const entity2D_ptr &e1,
                             const entity2D_ptr &e2,
                             const alg::vec2 &joint1,
                             const alg::vec2 &joint2,
                             const float length) : constraint2D<2>({e1, e2}),
                                                   m_length(length),
                                                   m_angle1(e1->angpos()),
                                                   m_angle2(e2->angpos()),
                                                   m_joint1(joint1),
                                                   m_joint2(joint2),
                                                   m_has_joints(true)
    {
    }

    float rigid_bar2D::constraint(const std::array<const_entity2D_ptr, 2> &entities) const
    {
        return m_has_joints ? with_joints_constraint(entities) : without_joints_constraint(entities);
    }

    float rigid_bar2D::constraint_derivative(const std::array<const_entity2D_ptr, 2> &entities) const
    {
        return m_has_joints ? with_joints_constraint_derivative(entities) : without_joints_constraint_derivative(entities);
    }

    float rigid_bar2D::without_joints_constraint(const std::array<const_entity2D_ptr, 2> &entities) const
    {
        const phys::const_entity2D_ptr &e1 = entities[0], &e2 = entities[1];
        return e1->pos().sq_dist(e2->pos()) - m_length * m_length;
    }
    float rigid_bar2D::without_joints_constraint_derivative(const std::array<const_entity2D_ptr, 2> &entities) const
    {
        const phys::const_entity2D_ptr &e1 = entities[0], &e2 = entities[1];
        return 2.f * (e1->pos() - e2->pos())
                         .dot(e1->vel() - e2->vel());
    }

    float rigid_bar2D::with_joints_constraint(const std::array<const_entity2D_ptr, 2> &entities) const
    {
        const phys::const_entity2D_ptr &e1 = entities[0], &e2 = entities[1];
        const alg::vec2 p1 = m_joint1.rotated(e1->angpos() - m_angle1) + e1->pos(),
                        p2 = m_joint2.rotated(e2->angpos() - m_angle2) + e2->pos();

        return p1.sq_dist(p2) - m_length * m_length;
    }
    float rigid_bar2D::with_joints_constraint_derivative(const std::array<const_entity2D_ptr, 2> &entities) const
    {
        const phys::const_entity2D_ptr &e1 = entities[0], &e2 = entities[1];
        const alg::vec2 rot_joint1 = m_joint1.rotated(e1->angpos() - m_angle1),
                        rot_joint2 = m_joint2.rotated(e2->angpos() - m_angle2);

        return 2.f * (rot_joint1 - rot_joint2 + e1->pos() - e2->pos())
                         .dot(e1->vel_at(rot_joint1) - e2->vel_at(rot_joint2));
    }

    std::array<float, 3> rigid_bar2D::constraint_grad(entity2D &e) const
    {
        const const_entity2D_ptr &e1 = m_entities[0], &e2 = m_entities[1];
        DBG_ASSERT(e == *e1 || e == *e2, "Passed entity to compute constraint gradient must be equal to some entity of the constraint!\n")
        if (!m_has_joints)
        {
            const alg::vec2 cg = 2.f * (e1->pos() - e2->pos());
            if (e == *e1)
                return {cg.x, cg.y, 0.f};
            return {-cg.x, -cg.y, 0.f};
        }
        const float a1 = e1->angpos() - m_angle1,
                    a2 = e2->angpos() - m_angle2;
        const alg::vec2 rot_joint1 = m_joint1.rotated(a1),
                        rot_joint2 = m_joint2.rotated(a2);
        const alg::vec2 cg = 2.f * (e1->pos() + rot_joint1 - e2->pos() - rot_joint2);
        if (e == *e1)
        {
            const float cga = -cg.x * (m_joint1.x * std::sinf(a1) + m_joint1.y * std::cosf(a1)) +
                              cg.y * (m_joint1.x * std::cosf(a1) - m_joint1.y * std::sinf(a1));
            return {cg.x, cg.y, cga};
        }
        const float cga = cg.x * (m_joint2.x * std::sinf(a2) + m_joint2.y * std::cosf(a2)) +
                          cg.y * (-m_joint2.x * std::cosf(a2) + m_joint2.y * std::sinf(a2));
        return {-cg.x, -cg.y, cga};
    }
    std::array<float, 3> rigid_bar2D::constraint_grad_derivative(entity2D &e) const
    {
        const const_entity2D_ptr &e1 = m_entities[0], &e2 = m_entities[1];
        DBG_ASSERT(e == *e1 || e == *e2, "Passed entity to compute constraint gradient must be equal to some entity of the constraint!\n")
        if (!m_has_joints)
        {
            const alg::vec2 cgd = 2.f * (e1->vel() - e2->vel());
            if (e == *e1)
                return {cgd.x, cgd.y, 0.f};
            return {-cgd.x, -cgd.y, 0.f};
        }
        const alg::vec2 rot_joint1 = m_joint1.rotated(e1->angpos() - m_angle1),
                        rot_joint2 = m_joint2.rotated(e2->angpos() - m_angle2);
        const alg::vec2 cgd = 2.f * (e1->vel_at(rot_joint1) - e2->vel_at(rot_joint2));
        if (e == *e1)
        {
            const float cgda = -cgd.x * rot_joint1.x - cgd.y * rot_joint1.y;
            return {cgd.x, cgd.y, cgda};
        }
        const float cgda = cgd.x * rot_joint2.x + cgd.y * rot_joint2.y;
        return {-cgd.x, -cgd.y, cgda};
    }

    void rigid_bar2D::write(ini::output &out) const
    {
        out.write("e1", m_entities[0]->index());
        out.write("e2", m_entities[1]->index());
        out.begin_section("joint1");
        joint1().write(out);
        out.end_section();
        out.begin_section("joint2");
        joint2().write(out);
        out.end_section();
        out.write("stiffness", stiffness());
        out.write("dampening", dampening());
        out.write("length", length());
        out.write("has_joints", m_has_joints);
    }

    void rigid_bar2D::read(ini::input &in)
    {
        stiffness(in.readf("stiffness"));
        dampening(in.readf("dampening"));
        length(in.readf("length"));
    }

    float rigid_bar2D::length() const { return m_length; }
    void rigid_bar2D::length(const float length) { m_length = length; }

    const_entity2D_ptr rigid_bar2D::e1() const { return m_entities[0]; }
    const_entity2D_ptr rigid_bar2D::e2() const { return m_entities[1]; }

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

    bool rigid_bar2D::has_joints() const { return m_has_joints; }
}