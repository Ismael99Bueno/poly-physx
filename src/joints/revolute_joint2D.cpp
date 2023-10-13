#include "ppx/internal/pch.hpp"
#include "ppx/joints/revolute_joint2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
revolute_joint2D::revolute_joint2D(const body2D::ptr &body1, const body2D::ptr &body2)
    : constraint2D("Revolute"), joint2D(body1, body2),
      m_length(glm::distance(body1->transform().position, body2->transform().position))
{
}

revolute_joint2D::revolute_joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1,
                                   const glm::vec2 &anchor2)
    : constraint2D("Revolute"), joint2D(body1, body2, anchor1, anchor2),
      m_length(glm::distance(body1->transform().position + anchor1, body2->transform().position + anchor2))
{
}
revolute_joint2D::revolute_joint2D(const specs &spc) : constraint2D("Revolute"), joint2D(spc)
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

std::pair<float, float> revolute_joint2D::compute_impulses() const
{
    const float ctr_deriv = constraint_derivative();
    if (!m_has_anchors)
    {
        const glm::vec2 relpos = m_body1->transform().position - m_body2->transform().position;
        const float cte = -ctr_deriv / (4.f * glm::length(relpos));
        return {cte * m_body1->mass(), cte * m_body2->mass()};
    }

    const glm::vec2 rot_anchor1 = rotated_anchor1();
    const glm::vec2 rot_anchor2 = rotated_anchor2();

    const glm::vec2 relpos = m_body1->transform().position + rot_anchor1 - m_body2->transform().position - rot_anchor2;
    const glm::vec2 dir = glm::normalize(relpos);

    const float dist = glm::length(relpos);

    const float f1 = 4.f * dist;
    const float f2 = glm::length2(rot_anchor1) * dist - glm::dot(rot_anchor1, dir) * glm::dot(rot_anchor1, relpos) +
                     glm::length2(rot_anchor2) * dist - glm::dot(rot_anchor2, dir) * glm::dot(rot_anchor2, relpos);

    const float imp1 = -ctr_deriv / (f1 * m_body1->inverse_mass() + 2.f * f2 * m_body1->inverse_inertia());
    const float imp2 = -ctr_deriv / (f1 * m_body2->inverse_mass() + 2.f * f2 * m_body2->inverse_inertia());
    return {imp1, imp2};
}

void revolute_joint2D::warmup()
{
    if (kit::approaches_zero(m_accumulated_impulse1) && kit::approaches_zero(m_accumulated_impulse2))
        return;

    if (m_has_anchors)
    {
        const glm::vec2 rot_anchor1 = rotated_anchor1();
        const glm::vec2 rot_anchor2 = rotated_anchor2();
        const glm::vec2 dir =
            glm::normalize(m_body1->transform().position + rot_anchor1 - m_body2->transform().position - rot_anchor2);

        aggregate_impulse(*m_body1, m_accumulated_impulse1 * dir, rot_anchor1);
        aggregate_impulse(*m_body2, -m_accumulated_impulse2 * dir, rot_anchor2);
    }
    else
    {
        const glm::vec2 dir = glm::normalize(m_body1->transform().position - m_body2->transform().position);
        aggregate_impulse(*m_body1, m_accumulated_impulse1 * dir);
        aggregate_impulse(*m_body2, -m_accumulated_impulse2 * dir);
    }
}
void revolute_joint2D::solve()
{
    const auto [imp1, imp2] = compute_impulses();
    m_accumulated_impulse1 += imp1;
    m_accumulated_impulse2 += imp2;

    if (m_has_anchors)
    {
        const glm::vec2 rot_anchor1 = rotated_anchor1();
        const glm::vec2 rot_anchor2 = rotated_anchor2();
        const glm::vec2 dir =
            glm::normalize(m_body1->transform().position + rot_anchor1 - m_body2->transform().position - rot_anchor2);

        aggregate_impulse(*m_body1, imp1 * dir, rot_anchor1);
        aggregate_impulse(*m_body2, -imp2 * dir, rot_anchor2);
    }
    else
    {
        const glm::vec2 dir = glm::normalize(m_body1->transform().position - m_body2->transform().position);
        aggregate_impulse(*m_body1, imp1 * dir);
        aggregate_impulse(*m_body2, -imp2 * dir);
    }
}

void revolute_joint2D::finalize(std::vector<float> &state_derivative)
{
    if (m_has_anchors)
    {
        const glm::vec2 rot_anchor1 = rotated_anchor1();
        const glm::vec2 rot_anchor2 = rotated_anchor2();
        const glm::vec2 dir =
            glm::normalize(m_body1->transform().position + rot_anchor1 - m_body2->transform().position - rot_anchor2);

        apply_impulse(*m_body1, m_accumulated_impulse1 * dir, rot_anchor1, state_derivative);
        apply_impulse(*m_body2, -m_accumulated_impulse2 * dir, rot_anchor2, state_derivative);
    }
    else
    {
        const glm::vec2 dir = glm::normalize(m_body1->transform().position - m_body2->transform().position);
        apply_impulse(*m_body1, m_accumulated_impulse1 * dir, state_derivative);
        apply_impulse(*m_body2, -m_accumulated_impulse2 * dir, state_derivative);
    }
    m_accumulated_impulse1 = 0.f;
    m_accumulated_impulse2 = 0.f;
}

bool revolute_joint2D::any_kinematic() const
{
    return m_body1->kinematic || m_body2->kinematic;
}

float revolute_joint2D::without_anchors_constraint() const
{
    return glm::distance2(m_body1->transform().position, m_body2->transform().position) - m_length * m_length;
}
float revolute_joint2D::without_anchors_constraint_derivative() const
{
    return 2.f * glm::dot(m_body1->transform().position - m_body2->transform().position,
                          m_body1->velocity() - m_body2->velocity());
}

float revolute_joint2D::with_anchors_constraint() const
{
    const glm::vec2 p1 = rotated_anchor1() + m_body1->transform().position,
                    p2 = rotated_anchor2() + m_body2->transform().position;

    return glm::distance2(p1, p2) - m_length * m_length;
}
float revolute_joint2D::with_anchors_constraint_derivative() const
{
    const glm::vec2 rot_anchor1 = rotated_anchor1(), rot_anchor2 = rotated_anchor2();

    return 2.f * glm::dot(rot_anchor1 - rot_anchor2 + m_body1->transform().position - m_body2->transform().position,
                          m_body1->velocity_at(rot_anchor1) - m_body2->velocity_at(rot_anchor2));
}

bool revolute_joint2D::valid() const
{
    return joint2D::valid();
}
bool revolute_joint2D::contains(const kit::uuid id) const
{
    return m_body1->id == id || m_body2->id == id;
}
float revolute_joint2D::length() const
{
    return m_length;
}

revolute_joint2D::specs revolute_joint2D::specs::from_revolute_joint(const revolute_joint2D &rj)
{
    return {rj.body1(), rj.body2(), rj.rotated_anchor1(), rj.rotated_anchor2(), rj.has_anchors()};
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