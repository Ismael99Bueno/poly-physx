#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint2D.hpp"

namespace ppx
{
joint2D::joint2D(const body2D::ptr &body1, const body2D::ptr &body2)
    : m_body1(body1), m_body2(body2), m_has_anchors(false)
{
}

joint2D::joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1, const glm::vec2 &anchor2)
    : m_body1(body1), m_body2(body2), m_anchor1(anchor1), m_anchor2(anchor2), m_angle1(body1->transform().rotation),
      m_angle2(body2->transform().rotation), m_has_anchors(true)
{
}
joint2D::joint2D(const specs &spc) : m_body1(spc.body1), m_body2(spc.body2), m_has_anchors(spc.has_anchors)
{
    if (m_has_anchors)
    {
        m_anchor1 = spc.anchor1;
        m_anchor2 = spc.anchor2;
        m_angle1 = m_body1->transform().rotation;
        m_angle2 = m_body2->transform().rotation;
    }
}

bool joint2D::valid() const
{
    return m_body1 && m_body2;
}

const body2D::ptr &joint2D::body1() const
{
    return m_body1;
}
const body2D::ptr &joint2D::body2() const
{
    return m_body2;
}

void joint2D::body1(const body2D::ptr &body1)
{
    m_body1 = body1;
    if (m_has_anchors)
        anchor1(rotated_anchor1());
}
void joint2D::body2(const body2D::ptr &body2)
{
    m_body2 = body2;
    if (m_has_anchors)
        anchor2(rotated_anchor2());
}

glm::vec2 joint2D::rotated_anchor1() const
{
    return glm::rotate(m_anchor1, m_body1->transform().rotation - m_angle1);
}
glm::vec2 joint2D::rotated_anchor2() const
{
    return glm::rotate(m_anchor2, m_body2->transform().rotation - m_angle2);
}

const glm::vec2 &joint2D::anchor1() const
{
    return m_anchor1;
}
const glm::vec2 &joint2D::anchor2() const
{
    return m_anchor2;
}

void joint2D::anchor1(const glm::vec2 &anchor1)
{
    m_anchor1 = anchor1;
    m_angle1 = m_body1->transform().rotation;
    m_has_anchors = true;
}
void joint2D::anchor2(const glm::vec2 &anchor2)
{
    m_anchor2 = anchor2;
    m_angle2 = m_body2->transform().rotation;
    m_has_anchors = true;
}

bool joint2D::has_anchors() const
{
    return m_has_anchors;
}

#ifdef KIT_USE_YAML_CPP
YAML::Node joint2D::encode() const
{
    YAML::Node node;
    node["Index1"] = m_body1->index;
    node["Index2"] = m_body2->index;
    if (m_has_anchors)
    {
        node["Anchor1"] = rotated_anchor1();
        node["Anchor2"] = rotated_anchor2();
    }
    return node;
}
bool joint2D::decode(const YAML::Node &node)
{
    return true;
}
#endif
} // namespace ppx