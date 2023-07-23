#include "ppx/internal/pch.hpp"
#include "ppx/joints/revolute_joint2D.hpp"

namespace ppx
{
revolute_joint2D::revolute_joint2D(const entity2D::ptr &e1, const entity2D::ptr &e2, const float stiffness,
                                   const float dampening)
    : constraint2D(stiffness, dampening), joint2D(e1, e2), m_length(glm::distance(e1->pos(), e2->pos()))
{
}

revolute_joint2D::revolute_joint2D(const entity2D::ptr &e1, const entity2D::ptr &e2, const glm::vec2 &anchor1,
                                   const glm::vec2 &anchor2, const float stiffness, const float dampening)
    : constraint2D(stiffness, dampening), joint2D(e1, e2, anchor1, anchor2),
      m_length(glm::distance(e1->pos() + anchor1, e2->pos() + anchor2))
{
}
revolute_joint2D::revolute_joint2D(const specs &spc) : constraint2D(spc.stiffness, spc.dampening), joint2D(spc)
{

    m_length = spc.has_anchors ? glm::distance(spc.e1->pos() + spc.anchor1, spc.e2->pos() + spc.anchor2)
                               : glm::distance(spc.e1->pos(), spc.e2->pos());
}

float revolute_joint2D::constraint_value() const
{
    return m_has_anchors ? with_anchors_constraint() : without_anchors_constraint();
}

float revolute_joint2D::constraint_derivative() const
{
    return m_has_anchors ? with_anchors_constraint_derivative() : without_anchors_constraint_derivative();
}

float revolute_joint2D::without_anchors_constraint() const
{
    return glm::distance2(m_e1->pos(), m_e2->pos()) - m_length * m_length;
}
float revolute_joint2D::without_anchors_constraint_derivative() const
{
    return 2.f * glm::dot(m_e1->pos() - m_e2->pos(), m_e1->vel() - m_e2->vel());
}

float revolute_joint2D::with_anchors_constraint() const
{
    const glm::vec2 p1 = anchor1() + m_e1->pos(), p2 = anchor2() + m_e2->pos();

    return glm::distance2(p1, p2) - m_length * m_length;
}
float revolute_joint2D::with_anchors_constraint_derivative() const
{
    const glm::vec2 rot_anchor1 = anchor1(), rot_anchor2 = anchor2();

    return 2.f * glm::dot(rot_anchor1 - rot_anchor2 + m_e1->pos() - m_e2->pos(),
                          m_e1->vel_at(rot_anchor1) - m_e2->vel_at(rot_anchor2));
}

std::vector<constraint2D::entity_gradient> revolute_joint2D::constraint_gradients() const
{
    if (!m_has_anchors)
    {
        const glm::vec2 cg = 2.f * (m_e1->pos() - m_e2->pos());
        return {{m_e1.raw(), {cg.x, cg.y, 0.f}}, {m_e2.raw(), {-cg.x, -cg.y, 0.f}}};
    }
    const glm::vec2 rot_anchor1 = anchor1(), rot_anchor2 = anchor2();
    const glm::vec2 cg = 2.f * (m_e1->pos() + rot_anchor1 - m_e2->pos() - rot_anchor2);

    const glm::vec2 perp_cg = {cg.y, -cg.x};
    const float cga1 = glm::dot(rot_anchor1, perp_cg);
    const float cga2 = glm::dot(rot_anchor2, -perp_cg);

    return {{m_e1.raw(), {cg.x, cg.y, cga1}}, {m_e2.raw(), {-cg.x, -cg.y, cga2}}};
}
std::vector<constraint2D::entity_gradient> revolute_joint2D::constraint_derivative_gradients() const
{
    if (!m_has_anchors)
    {
        const glm::vec2 cgd = 2.f * (m_e1->vel() - m_e2->vel());
        return {{m_e1.raw(), {cgd.x, cgd.y, 0.f}}, {m_e2.raw(), {-cgd.x, -cgd.y, 0.f}}};
    }
    const glm::vec2 rot_anchor1 = anchor1(), rot_anchor2 = anchor2();
    const glm::vec2 cgd = 2.f * (m_e1->vel_at(rot_anchor1) - m_e2->vel_at(rot_anchor2));

    const float cgda1 = glm::dot(rot_anchor1, -cgd);
    const float cgda2 = glm::dot(rot_anchor2, cgd);

    return {{m_e1.raw(), {cgd.x, cgd.y, cgda1}}, {m_e2.raw(), {-cgd.x, -cgd.y, cgda2}}};
}

bool revolute_joint2D::valid() const
{
    return joint2D::valid();
}
float revolute_joint2D::length() const
{
    return m_length;
}

revolute_joint2D::specs revolute_joint2D::specs::from_rigid_bar(const revolute_joint2D &rb)
{
    return {{rb.e1(), rb.e2(), rb.anchor1(), rb.anchor2(), rb.has_anchors()}, rb.stiffness(), rb.dampening()};
}

#ifdef KIT_USE_YAML_CPP
YAML::Node revolute_joint2D::encode() const
{
    const YAML::Node node1 = joint2D::encode();
    const YAML::Node node2 = constraint2D::encode();

    YAML::Node node;
    node["Joint2D"] = node1;
    node["Constraint2D"] = node2;

    return node;
}
bool revolute_joint2D::decode(const YAML::Node &node)
{
    if (!node.IsMap() || node.size() != 2)
        return false;

    if (!joint2D::decode(node["Joint2D"]))
        return false;
    if (!constraint2D::decode(node["Constraint2D"]))
        return false;
    return true;
}
#endif
} // namespace ppx