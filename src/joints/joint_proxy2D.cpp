#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint_proxy2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{

joint_proxy2D::joint_proxy2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1,
                             const glm::vec2 &anchor2)
    : m_body1(body1), m_body2(body2), m_anchor1(anchor1), m_anchor2(anchor2), m_angle1(body1->rotation()),
      m_angle2(body2->rotation())
{
}
joint_proxy2D::joint_proxy2D(const specs &spc) : joint_proxy2D(spc.body1, spc.body2, spc.anchor1, spc.anchor2)
{
}

bool joint_proxy2D::valid() const
{
    return m_body1 && m_body2;
}

const body2D::ptr &joint_proxy2D::body1() const
{
    return m_body1;
}
const body2D::ptr &joint_proxy2D::body2() const
{
    return m_body2;
}

void joint_proxy2D::body1(const body2D::ptr &body1)
{
    m_body1 = body1;
    anchor1(rotated_anchor1());
}
void joint_proxy2D::body2(const body2D::ptr &body2)
{
    m_body2 = body2;
    anchor2(rotated_anchor2());
}

glm::vec2 joint_proxy2D::rotated_anchor1() const
{
    return glm::rotate(m_anchor1, m_body1->rotation() - m_angle1);
}
glm::vec2 joint_proxy2D::rotated_anchor2() const
{
    return glm::rotate(m_anchor2, m_body2->rotation() - m_angle2);
}

const glm::vec2 &joint_proxy2D::anchor1() const
{
    return m_anchor1;
}
const glm::vec2 &joint_proxy2D::anchor2() const
{
    return m_anchor2;
}

void joint_proxy2D::anchor1(const glm::vec2 &anchor1)
{
    m_anchor1 = anchor1;
    m_angle1 = m_body1->rotation();
}
void joint_proxy2D::anchor2(const glm::vec2 &anchor2)
{
    m_anchor2 = anchor2;
    m_angle2 = m_body2->rotation();
}

#ifdef KIT_USE_YAML_CPP
YAML::Node joint_proxy2D::encode() const
{
    YAML::Node node;
    node["Index1"] = m_body1->index;
    node["Index2"] = m_body2->index;

    node["Anchor1"] = rotated_anchor1();
    node["Anchor2"] = rotated_anchor2();

    return node;
}
bool joint_proxy2D::decode(const YAML::Node &node, world2D &world)
{
    if (!node.IsMap() || node.size() < 2)
        return false;
    const std::size_t index1 = node["Index1"].as<std::size_t>();
    const std::size_t index2 = node["Index2"].as<std::size_t>();

    const glm::vec2 a1 = node["Anchor1"].as<glm::vec2>();
    const glm::vec2 a2 = node["Anchor2"].as<glm::vec2>();

    m_body1 = world[index1];
    m_body2 = world[index2];

    anchor1(a1);
    anchor2(a2);
    return true;
}
#endif
} // namespace ppx