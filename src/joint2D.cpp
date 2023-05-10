#include "ppx/pch.hpp"
#include "ppx/joint2D.hpp"

namespace ppx
{
    joint2D::joint2D(const entity2D_ptr &e1,
                     const entity2D_ptr &e2,
                     const float length) : m_e1(e1),
                                           m_e2(e2),
                                           m_length(length),
                                           m_has_anchors(false) {}

    joint2D::joint2D(const entity2D_ptr &e1,
                     const entity2D_ptr &e2,
                     const glm::vec2 &anchor1,
                     const glm::vec2 &anchor2,
                     const float length) : m_e1(e1),
                                           m_e2(e2),
                                           m_anchor1(anchor1),
                                           m_anchor2(anchor2),
                                           m_angle1(e1->angpos()),
                                           m_angle2(e2->angpos()),
                                           m_length(length),
                                           m_has_anchors(true) {}

    void joint2D::bind(const entity2D_ptr &e1, const entity2D_ptr &e2)
    {
        m_e1 = e1;
        m_e2 = e2;
        if (m_has_anchors)
        {
            anchor1(anchor1());
            anchor2(anchor2());
        }
    }
    bool joint2D::validate() { return m_e1.validate() && m_e2.validate(); }

    float joint2D::length() const { return m_length; }
    void joint2D::length(const float length) { m_length = length; }

    const entity2D_ptr &joint2D::e1() const { return m_e1; }
    const entity2D_ptr &joint2D::e2() const { return m_e2; }

    glm::vec2 joint2D::anchor1() const { return glm::rotate(m_anchor1, m_e1->angpos() - m_angle1); }
    glm::vec2 joint2D::anchor2() const { return glm::rotate(m_anchor2, m_e2->angpos() - m_angle2); }

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

    bool joint2D::has_anchors() const { return m_has_anchors; }
    joint2D::specs joint2D::specs::from_joint(const joint2D &joint)
    {
        return {joint.e1(), joint.e2(), joint.anchor1(), joint.anchor2(), joint.length(), joint.has_anchors()};
    }

#ifdef HAS_YAML_CPP
    void joint2D::write(YAML::Emitter &out) const
    {
        out << YAML::Key << "ID1" << YAML::Value << (std::uint64_t)m_e1.id();
        out << YAML::Key << "ID2" << YAML::Value << (std::uint64_t)m_e2.id();
        out << YAML::Key << "Index1" << YAML::Value << m_e1.index();
        out << YAML::Key << "Index2" << YAML::Value << m_e2.index();
        if (m_has_anchors)
        {
            out << YAML::Key << "Anchor1" << YAML::Value << anchor1();
            out << YAML::Key << "Anchor2" << YAML::Value << anchor2();
        }
        out << YAML::Key << "length" << YAML::Value << m_length;
    }
    YAML::Node joint2D::encode() const
    {
        YAML::Node node;
        node["ID1"] = (std::uint64_t)m_e1.id();
        node["ID2"] = (std::uint64_t)m_e2.id();
        node["Index1"] = m_e1.index();
        node["Index2"] = m_e2.index();
        if (m_has_anchors)
        {
            node["Anchor1"] = anchor1();
            node["Anchor2"] = anchor2();
        }
        node["length"] = m_length;
        return node;
    }
    bool joint2D::decode(const YAML::Node &node)
    {
        if (!node.IsMap() || node.size() < 5)
            return false;
        m_length = node["length"].as<float>();
        return true;
    }
#endif
#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const joint2D &joint)
    {
        out << YAML::BeginMap;
        joint.write(out);
        out << YAML::EndMap;
        return out;
    }
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    Node convert<ppx::joint2D>::encode(const ppx::joint2D &joint)
    {
        return joint.encode();
    }
    bool convert<ppx::joint2D>::decode(const Node &node, ppx::joint2D &joint)
    {
        return joint.decode(node);
    };
}
#endif