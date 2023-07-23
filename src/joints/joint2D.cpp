#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint2D.hpp"

namespace ppx
{
joint2D::joint2D(const body2D::ptr &body1, const body2D::ptr &body2) : m_e1(body1), m_e2(body2), m_has_anchors(false)
{
}

joint2D::joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1, const glm::vec2 &anchor2)
    : m_e1(body1), m_e2(body2), m_anchor1(anchor1), m_anchor2(anchor2), m_angle1(body1->rotation()),
      m_angle2(body2->rotation()), m_has_anchors(true)
{
}
joint2D::joint2D(const specs &spc) : m_e1(spc.body1), m_e2(spc.body2), m_has_anchors(spc.has_anchors)
{
    if (m_has_anchors)
    {
        m_anchor1 = spc.anchor1;
        m_anchor2 = spc.anchor2;
        m_angle1 = m_e1->rotation();
        m_angle2 = m_e2->rotation();
    }
}

void joint2D::bind(const body2D::ptr &body1, const body2D::ptr &body2)
{
    m_e1 = body1;
    m_e2 = body2;
    if (m_has_anchors)
    {
        anchor1(anchor1());
        anchor2(anchor2());
    }
}
bool joint2D::valid() const
{
    return m_e1 && m_e2;
}

const body2D::ptr &joint2D::body1() const
{
    return m_e1;
}
const body2D::ptr &joint2D::body2() const
{
    return m_e2;
}

glm::vec2 joint2D::anchor1() const
{
    return glm::rotate(m_anchor1, m_e1->rotation() - m_angle1);
}
glm::vec2 joint2D::anchor2() const
{
    return glm::rotate(m_anchor2, m_e2->rotation() - m_angle2);
}

void joint2D::anchor1(const glm::vec2 &anchor1)
{
    m_anchor1 = anchor1;
    m_angle1 = m_e1->rotation();
    m_has_anchors = true;
}
void joint2D::anchor2(const glm::vec2 &anchor2)
{
    m_anchor2 = anchor2;
    m_angle2 = m_e2->rotation();
    m_has_anchors = true;
}

bool joint2D::has_anchors() const
{
    return m_has_anchors;
}

joint2D::specs joint2D::specs::from_joint(const joint2D &joint)
{
    return {joint.body1(), joint.body2(), joint.anchor1(), joint.anchor2(), joint.has_anchors()};
}

#ifdef KIT_USE_YAML_CPP
YAML::Node joint2D::encode() const
{
    YAML::Node node;
    node["Index1"] = m_e1->index();
    node["Index2"] = m_e2->index();
    if (m_has_anchors)
    {
        node["Anchor1"] = anchor1();
        node["Anchor2"] = anchor2();
    }
    return node;
}
bool joint2D::decode(const YAML::Node &node)
{
    if (!node.IsMap() || node.size() < 4)
        return false;
    return true;
}
#endif
} // namespace ppx