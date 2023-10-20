#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint2D.hpp"

namespace ppx
{

joint2D::joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1, const glm::vec2 &anchor2)
    : m_body1(body1), m_body2(body2), m_anchor1(anchor1), m_anchor2(anchor2), m_angle1(body1->rotation()),
      m_angle2(body2->rotation())
{
}
joint2D::joint2D(const specs &spc) : joint2D(spc.body1, spc.body2, spc.anchor1, spc.anchor2)
{
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
    anchor1(rotated_anchor1());
}
void joint2D::body2(const body2D::ptr &body2)
{
    m_body2 = body2;
    anchor2(rotated_anchor2());
}

glm::vec2 joint2D::rotated_anchor1() const
{
    return glm::rotate(m_anchor1, m_body1->rotation() - m_angle1);
}
glm::vec2 joint2D::rotated_anchor2() const
{
    return glm::rotate(m_anchor2, m_body2->rotation() - m_angle2);
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
    m_angle1 = m_body1->rotation();
}
void joint2D::anchor2(const glm::vec2 &anchor2)
{
    m_anchor2 = anchor2;
    m_angle2 = m_body2->rotation();
}

#ifdef KIT_USE_YAML_CPP
YAML::Node joint2D::encode() const
{
    YAML::Node node;
    node["Index1"] = m_body1->index;
    node["Index2"] = m_body2->index;

    node["Anchor1"] = rotated_anchor1();
    node["Anchor2"] = rotated_anchor2();

    return node;
}
bool joint2D::decode(const YAML::Node &node)
{
    return true;
}
#endif
} // namespace ppx