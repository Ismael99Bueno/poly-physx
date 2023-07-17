#include "ppx/internal/pch.hpp"
#include "ppx/joints/revolute_joint2D.hpp"

namespace ppx
{
revolute_joint2D::revolute_joint2D(const entity2D::ptr &e1, const entity2D::ptr &e2, const float stiffness,
                                   const float dampening)
    : constraint2D<2>({e1, e2}, stiffness, dampening), joint2D(e1, e2, glm::distance(e1->pos(), e2->pos()))
{
}

revolute_joint2D::revolute_joint2D(const entity2D::ptr &e1, const entity2D::ptr &e2, const glm::vec2 &anchor1,
                                   const glm::vec2 &anchor2, const float stiffness, const float dampening)
    : constraint2D<2>({e1, e2}, stiffness, dampening),
      joint2D(e1, e2, anchor1, anchor2, glm::distance(e1->pos() + anchor1, e2->pos() + anchor2))
{
}
revolute_joint2D::revolute_joint2D(const specs &spc)
    : constraint2D<2>({spc.e1, spc.e2}, spc.stiffness, spc.dampening), joint2D(spc)
{
}

float revolute_joint2D::constraint(const std::array<entity2D::const_ptr, 2> &entities) const
{
    return m_has_anchors ? with_anchors_constraint(entities) : without_anchors_constraint(entities);
}

float revolute_joint2D::constraint_derivative(const std::array<entity2D::const_ptr, 2> &entities) const
{
    return m_has_anchors ? with_anchors_constraint_derivative(entities)
                         : without_anchors_constraint_derivative(entities);
}

float revolute_joint2D::without_anchors_constraint(const std::array<entity2D::const_ptr, 2> &entities) const
{
    return glm::distance2(m_e1->pos(), m_e2->pos()) - m_length * m_length;
}
float revolute_joint2D::without_anchors_constraint_derivative(const std::array<entity2D::const_ptr, 2> &entities) const
{
    return 2.f * glm::dot(m_e1->pos() - m_e2->pos(), m_e1->vel() - m_e2->vel());
}

float revolute_joint2D::with_anchors_constraint(const std::array<entity2D::const_ptr, 2> &entities) const
{
    const glm::vec2 p1 = anchor1() + m_e1->pos(), p2 = anchor2() + m_e2->pos();

    return glm::distance2(p1, p2) - m_length * m_length;
}
float revolute_joint2D::with_anchors_constraint_derivative(const std::array<entity2D::const_ptr, 2> &entities) const
{
    const glm::vec2 rot_anchor1 = anchor1(), rot_anchor2 = anchor2();

    return 2.f * glm::dot(rot_anchor1 - rot_anchor2 + m_e1->pos() - m_e2->pos(),
                          m_e1->vel_at(rot_anchor1) - m_e2->vel_at(rot_anchor2));
}

std::array<float, 3> revolute_joint2D::constraint_grad(entity2D &e) const
{
    KIT_ASSERT_CRITICAL(e == *m_e1 || e == *m_e2,
                        "Passed entity to compute constraint gradient must be equal to some entity of the constraint!")
    if (!m_has_anchors)
    {
        const glm::vec2 cg = 2.f * (m_e1->pos() - m_e2->pos());
        if (e == *m_e1)
            return {cg.x, cg.y, 0.f};
        return {-cg.x, -cg.y, 0.f};
    }
    const float a1 = m_e1->angpos() - m_angle1, a2 = m_e2->angpos() - m_angle2;
    const glm::vec2 rot_anchor1 = glm::rotate(m_anchor1, a1), rot_anchor2 = glm::rotate(m_anchor2, a2);
    const glm::vec2 cg = 2.f * (m_e1->pos() + rot_anchor1 - m_e2->pos() - rot_anchor2);
    if (e == *m_e1)
    {
        const float cos = cosf(a1), sin = sinf(a1);
        const float cga =
            -cg.x * (m_anchor1.x * sin + m_anchor1.y * cos) + cg.y * (m_anchor1.x * cos - m_anchor1.y * sin);
        return {cg.x, cg.y, cga};
    }
    const float cos = cosf(a2), sin = sinf(a2);
    const float cga = cg.x * (m_anchor2.x * sin + m_anchor2.y * cos) + cg.y * (-m_anchor2.x * cos + m_anchor2.y * sin);
    return {-cg.x, -cg.y, cga};
}
std::array<float, 3> revolute_joint2D::constraint_grad_derivative(entity2D &e) const
{
    KIT_ASSERT_CRITICAL(e == *m_e1 || e == *m_e2,
                        "Passed entity to compute constraint gradient must be equal to some entity of the constraint!")
    if (!m_has_anchors)
    {
        const glm::vec2 cgd = 2.f * (m_e1->vel() - m_e2->vel());
        if (e == *m_e1)
            return {cgd.x, cgd.y, 0.f};
        return {-cgd.x, -cgd.y, 0.f};
    }
    const glm::vec2 rot_anchor1 = anchor1(), rot_anchor2 = anchor2();
    const glm::vec2 cgd = 2.f * (m_e1->vel_at(rot_anchor1) - m_e2->vel_at(rot_anchor2));
    if (e == *m_e1)
    {
        const float cgda = -cgd.x * rot_anchor1.x - cgd.y * rot_anchor1.y;
        return {cgd.x, cgd.y, cgda};
    }
    const float cgda = cgd.x * rot_anchor2.x + cgd.y * rot_anchor2.y;
    return {-cgd.x, -cgd.y, cgda};
}

bool revolute_joint2D::valid() const
{
    return constraint2D::valid() && joint2D::valid();
}
revolute_joint2D::specs revolute_joint2D::specs::from_rigid_bar(const revolute_joint2D &rb)
{
    return {
        {rb.e1(), rb.e2(), rb.anchor1(), rb.anchor2(), rb.length(), rb.has_anchors()}, rb.stiffness(), rb.dampening()};
}

void revolute_joint2D::write(YAML::Emitter &out) const
{
    out << YAML::Key << "UUID" << YAML::Value << (std::uint64_t)id();
    joint2D::write(out);
    out << YAML::Key << "Stiffness" << YAML::Value << m_stiffness;
    out << YAML::Key << "Dampening" << YAML::Value << m_dampening;
}
YAML::Node revolute_joint2D::encode() const
{
    YAML::Node node = joint2D::encode();
    node["UUID"] = (std::uint64_t)id();
    node["Stiffness"] = m_stiffness;
    node["Dampening"] = m_dampening;
    return node;
}
bool revolute_joint2D::decode(const YAML::Node &node)
{
    if (!joint2D::decode(node))
        return false;
    id(node["UUID"].as<std::uint64_t>());
    m_stiffness = node["Stiffness"].as<float>();
    m_dampening = node["Dampening"].as<float>();
    return true;
}
} // namespace ppx

#ifdef YAML_CPP_COMPAT
namespace YAML
{
Node convert<ppx::revolute_joint2D>::encode(const ppx::revolute_joint2D &rb)
{
    return rb.encode();
}
bool convert<ppx::revolute_joint2D>::decode(const Node &node, ppx::revolute_joint2D &rb)
{
    return rb.decode(node);
};
} // namespace YAML
#endif