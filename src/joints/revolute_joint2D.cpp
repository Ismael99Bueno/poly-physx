#include "ppx/internal/pch.hpp"
#include "ppx/joints/revolute_joint2D.hpp"

namespace ppx
{
revolute_joint2D::revolute_joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const float stiffness,
                                   const float dampening)
    : constraint2D("Revolute", stiffness, dampening), joint2D(body1, body2),
      m_length(glm::distance(body1->transform().position, body2->transform().position))
{
}

revolute_joint2D::revolute_joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1,
                                   const glm::vec2 &anchor2, const float stiffness, const float dampening)
    : constraint2D("Revolute", stiffness, dampening), joint2D(body1, body2, anchor1, anchor2),
      m_length(glm::distance(body1->transform().position + anchor1, body2->transform().position + anchor2))
{
}
revolute_joint2D::revolute_joint2D(const specs &spc)
    : constraint2D("Revolute", spc.stiffness, spc.dampening), joint2D(spc)
{

    m_length = spc.has_anchors ? glm::distance(spc.body1->transform().position + spc.anchor1,
                                               spc.body2->transform().position + spc.anchor2)
                               : glm::distance(spc.body1->transform().position, spc.body2->transform().position);
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
    return glm::distance2(m_e1->transform().position, m_e2->transform().position) - m_length * m_length;
}
float revolute_joint2D::without_anchors_constraint_derivative() const
{
    return 2.f * glm::dot(m_e1->transform().position - m_e2->transform().position, m_e1->velocity() - m_e2->velocity());
}

float revolute_joint2D::with_anchors_constraint() const
{
    const glm::vec2 p1 = anchor1() + m_e1->transform().position, p2 = anchor2() + m_e2->transform().position;

    return glm::distance2(p1, p2) - m_length * m_length;
}
float revolute_joint2D::with_anchors_constraint_derivative() const
{
    const glm::vec2 rot_anchor1 = anchor1(), rot_anchor2 = anchor2();

    return 2.f * glm::dot(rot_anchor1 - rot_anchor2 + m_e1->transform().position - m_e2->transform().position,
                          m_e1->velocity_at(rot_anchor1) - m_e2->velocity_at(rot_anchor2));
}

std::vector<constraint2D::body_gradient> revolute_joint2D::constraint_gradients() const
{
    if (!m_has_anchors)
    {
        const glm::vec2 cg = 2.f * (m_e1->transform().position - m_e2->transform().position);
        return {{m_e1.raw(), {cg.x, cg.y, 0.f}}, {m_e2.raw(), {-cg.x, -cg.y, 0.f}}};
    }
    const glm::vec2 rot_anchor1 = anchor1(), rot_anchor2 = anchor2();
    const glm::vec2 cg = 2.f * (m_e1->transform().position + rot_anchor1 - m_e2->transform().position - rot_anchor2);

    const glm::vec2 perp_cg = {cg.y, -cg.x};
    const float cga1 = glm::dot(rot_anchor1, perp_cg);
    const float cga2 = glm::dot(rot_anchor2, -perp_cg);

    return {{m_e1.raw(), {cg.x, cg.y, cga1}}, {m_e2.raw(), {-cg.x, -cg.y, cga2}}};
}
std::vector<constraint2D::body_gradient> revolute_joint2D::constraint_derivative_gradients() const
{
    if (!m_has_anchors)
    {
        const glm::vec2 cgd = 2.f * (m_e1->velocity() - m_e2->velocity());
        return {{m_e1.raw(), {cgd.x, cgd.y, 0.f}}, {m_e2.raw(), {-cgd.x, -cgd.y, 0.f}}};
    }
    const glm::vec2 rot_anchor1 = anchor1(), rot_anchor2 = anchor2();
    const glm::vec2 cgd = 2.f * (m_e1->velocity_at(rot_anchor1) - m_e2->velocity_at(rot_anchor2));

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
    return {{rb.body1(), rb.body2(), rb.anchor1(), rb.anchor2(), rb.has_anchors()}, rb.stiffness(), rb.dampening()};
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