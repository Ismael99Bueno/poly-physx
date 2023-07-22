#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint2D.hpp"

namespace ppx
{
joint2D::joint2D(const entity2D::ptr &e1, const entity2D::ptr &e2) : m_e1(e1), m_e2(e2), m_has_anchors(false)
{
}

joint2D::joint2D(const entity2D::ptr &e1, const entity2D::ptr &e2, const glm::vec2 &anchor1, const glm::vec2 &anchor2)
    : m_e1(e1), m_e2(e2), m_anchor1(anchor1), m_anchor2(anchor2), m_angle1(e1->angpos()), m_angle2(e2->angpos()),
      m_has_anchors(true)
{
}
joint2D::joint2D(const specs &spc) : m_e1(spc.e1), m_e2(spc.e2), m_has_anchors(spc.has_anchors)
{
    if (m_has_anchors)
    {
        m_anchor1 = spc.anchor1;
        m_anchor2 = spc.anchor2;
        m_angle1 = m_e1->angpos();
        m_angle2 = m_e2->angpos();
    }
}

void joint2D::bind(const entity2D::ptr &e1, const entity2D::ptr &e2)
{
    m_e1 = e1;
    m_e2 = e2;
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

const entity2D::ptr &joint2D::e1() const
{
    return m_e1;
}
const entity2D::ptr &joint2D::e2() const
{
    return m_e2;
}

glm::vec2 joint2D::anchor1() const
{
    return glm::rotate(m_anchor1, m_e1->angpos() - m_angle1);
}
glm::vec2 joint2D::anchor2() const
{
    return glm::rotate(m_anchor2, m_e2->angpos() - m_angle2);
}

void joint2D::anchor1(const glm::vec2 &anchor1)
{
    m_anchor1 = anchor1;
    m_angle1 = m_e1->angpos();
    m_has_anchors = true;
}
void joint2D::anchor2(const glm::vec2 &anchor2)
{
    m_anchor2 = anchor2;
    m_angle2 = m_e2->angpos();
    m_has_anchors = true;
}

bool joint2D::has_anchors() const
{
    return m_has_anchors;
}

joint2D::specs joint2D::specs::from_joint(const joint2D &joint)
{
    return {joint.e1(), joint.e2(), joint.anchor1(), joint.anchor2(), joint.has_anchors()};
}

#ifdef KIT_USE_YAML_CPP
YAML::Node joint2D::encode() const
{
    YAML::Node node;
    node["ID1"] = (std::uint64_t)m_e1->id();
    node["ID2"] = (std::uint64_t)m_e2->id();
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