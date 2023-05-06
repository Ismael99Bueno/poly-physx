#include "ppx/pch.hpp"
#include "ppx/spring2D.hpp"

namespace ppx
{
    static float cross(const glm::vec2 &v1, const glm::vec2 &v2) { return v1.x * v2.y - v1.y * v2.x; }
    spring2D::spring2D(const entity2D_ptr &e1,
                       const entity2D_ptr &e2,
                       const float stiffness,
                       const float dampening,
                       const float length) : m_e1(e1),
                                             m_e2(e2),
                                             m_stiffness(stiffness),
                                             m_dampening(dampening),
                                             m_length(length),
                                             m_has_anchors(false) {}

    spring2D::spring2D(const entity2D_ptr &e1,
                       const entity2D_ptr &e2,
                       const glm::vec2 &anchor1,
                       const glm::vec2 &anchor2,
                       const float stiffness,
                       const float dampening,
                       const float length) : m_e1(e1),
                                             m_e2(e2),
                                             m_anchor1(anchor1),
                                             m_anchor2(anchor2),
                                             m_stiffness(stiffness),
                                             m_dampening(dampening),
                                             m_angle1(e1->angpos()),
                                             m_angle2(e2->angpos()),
                                             m_length(length),
                                             m_has_anchors(true) {}

    std::tuple<glm::vec2, float, float> spring2D::force() const
    {
        return m_has_anchors ? with_anchors_force() : without_anchors_force();
    }

    std::tuple<glm::vec2, float, float> spring2D::without_anchors_force() const
    {
        const glm::vec2 relpos = m_e2->pos() - m_e1->pos(),
                        direction = glm::normalize(relpos),
                        relvel = direction * glm::dot(m_e2->vel() - m_e1->vel(), direction),
                        vlen = m_length * direction;
        return {m_stiffness * (relpos - vlen) + m_dampening * relvel, 0.f, 0.f};
    }

    std::tuple<glm::vec2, float, float> spring2D::with_anchors_force() const
    {
        const glm::vec2 rot_anchor1 = anchor1(),
                        rot_anchor2 = anchor2();
        const glm::vec2 p1 = m_e1->pos() + rot_anchor1,
                        p2 = m_e2->pos() + rot_anchor2;
        const glm::vec2 relpos = p2 - p1,
                        direction = glm::normalize(relpos),
                        relvel = direction * glm::dot(m_e2->vel_at(rot_anchor2) - m_e1->vel_at(rot_anchor1), direction),
                        vlen = m_length * direction;

        const glm::vec2 force = m_stiffness * (relpos - vlen) + m_dampening * relvel;
        const float torque1 = cross(rot_anchor1, force), torque2 = cross(force, rot_anchor2);
        return {force, torque1, torque2};
    }

    void spring2D::bind(const entity2D_ptr &e1, const entity2D_ptr &e2)
    {
        m_e1 = e1;
        m_e2 = e2;
        if (m_has_anchors)
        {
            anchor1(anchor1());
            anchor2(anchor2());
        }
    }
    bool spring2D::try_validate() { return m_e1.try_validate() && m_e2.try_validate(); }

    float spring2D::stiffness() const { return m_stiffness; }
    float spring2D::dampening() const { return m_dampening; }
    float spring2D::length() const { return m_length; }

    void spring2D::stiffness(const float stiffness) { m_stiffness = stiffness; }
    void spring2D::dampening(const float dampening) { m_dampening = dampening; }
    void spring2D::length(const float length) { m_length = length; }

    float spring2D::kinetic_energy() const { return m_e1->kinetic_energy() + m_e2->kinetic_energy(); }
    float spring2D::potential_energy() const
    {
        const glm::vec2 p1 = m_e1->pos() + anchor1(),
                        p2 = m_e2->pos() + anchor2();
        const float dist = glm::distance(p1, p2) - m_length;
        return 0.5f * m_stiffness * dist * dist;
    }
    float spring2D::energy() const { return kinetic_energy() + potential_energy(); }

    const entity2D_ptr &spring2D::e1() const { return m_e1; }
    const entity2D_ptr &spring2D::e2() const { return m_e2; }

    glm::vec2 spring2D::anchor1() const { return glm::rotate(m_anchor1, m_e1->angpos() - m_angle1); }
    glm::vec2 spring2D::anchor2() const { return glm::rotate(m_anchor2, m_e2->angpos() - m_angle2); }

    void spring2D::anchor1(const glm::vec2 &anchor1)
    {
        m_anchor1 = anchor1;
        m_angle1 = m_e1->angpos();
        m_has_anchors = true;
    }
    void spring2D::anchor2(const glm::vec2 &anchor2)
    {
        m_anchor2 = anchor2;
        m_angle2 = m_e2->angpos();
        m_has_anchors = true;
    }

    bool spring2D::has_anchors() const { return m_has_anchors; }
#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const spring2D &sp)
    {
        out << YAML::BeginMap;
        out << YAML::Key << "ID1" << YAML::Value << sp.e1().id();
        out << YAML::Key << "ID2" << YAML::Value << sp.e2().id();
        out << YAML::Key << "Index1" << YAML::Value << sp.e1().index();
        out << YAML::Key << "Index2" << YAML::Value << sp.e2().index();
        if (sp.has_anchors())
        {
            out << YAML::Key << "Anchor1" << YAML::Value << sp.anchor1();
            out << YAML::Key << "Anchor2" << YAML::Value << sp.anchor2();
        }
        out << YAML::Key << "Stiffness" << YAML::Value << sp.stiffness();
        out << YAML::Key << "Dampening" << YAML::Value << sp.dampening();
        out << YAML::Key << "length" << YAML::Value << sp.length();
        out << YAML::EndMap;
        return out;
    }
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    Node convert<ppx::spring2D>::encode(const ppx::spring2D &sp)
    {
        Node node;
        node["ID1"] = sp.e1().id();
        node["ID2"] = sp.e2().id();
        node["Index1"] = sp.e1().index();
        node["Index2"] = sp.e2().index();
        if (sp.has_anchors())
        {
            node["Anchor1"] = sp.anchor1();
            node["Anchor2"] = sp.anchor2();
        }
        node["Stiffness"] = sp.stiffness();
        node["Dampening"] = sp.dampening();
        node["length"] = sp.length();
        return node;
    }
    bool convert<ppx::spring2D>::decode(const Node &node, ppx::spring2D &sp)
    {
        if (!node.IsMap() || (node.size() != 7 && node.size() != 9))
            return false;

        sp.stiffness(node["Stiffness"].as<float>());
        sp.dampening(node["Dampening"].as<float>());
        sp.length(node["length"].as<float>());

        return true;
    };
}
#endif
