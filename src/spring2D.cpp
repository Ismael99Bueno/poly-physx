#include "ppx/pch.hpp"
#include "ppx/spring2D.hpp"

namespace ppx
{
    static float cross(const glm::vec2 &v1, const glm::vec2 &v2) { return v1.x * v2.y - v1.y * v2.x; }
    spring2D::spring2D(const entity2D_ptr &e1,
                       const entity2D_ptr &e2,
                       const float stiffness,
                       const float dampening,
                       const float length) : joint2D(e1, e2, length),
                                             m_stiffness(stiffness),
                                             m_dampening(dampening) {}

    spring2D::spring2D(const entity2D_ptr &e1,
                       const entity2D_ptr &e2,
                       const glm::vec2 &anchor1,
                       const glm::vec2 &anchor2,
                       const float stiffness,
                       const float dampening,
                       const float length) : joint2D(e1, e2, anchor1, anchor2, length),
                                             m_stiffness(stiffness),
                                             m_dampening(dampening) {}
    spring2D::spring2D(const specs &spc) : joint2D(spc), m_stiffness(spc.stiffness),
                                           m_dampening(spc.dampening) {}

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

    float spring2D::stiffness() const { return m_stiffness; }
    float spring2D::dampening() const { return m_dampening; }

    void spring2D::stiffness(const float stiffness) { m_stiffness = stiffness; }
    void spring2D::dampening(const float dampening) { m_dampening = dampening; }

    float spring2D::kinetic_energy() const { return m_e1->kinetic_energy() + m_e2->kinetic_energy(); }
    float spring2D::potential_energy() const
    {
        const glm::vec2 p1 = m_e1->pos() + anchor1(),
                        p2 = m_e2->pos() + anchor2();
        const float dist = glm::distance(p1, p2) - m_length;
        return 0.5f * m_stiffness * dist * dist;
    }
    float spring2D::energy() const { return kinetic_energy() + potential_energy(); }
    spring2D::specs spring2D::specs::from_spring(const spring2D &sp)
    {
        return {{sp.e1(), sp.e2(), sp.anchor1(), sp.anchor2(), sp.length(), sp.has_anchors()}, sp.stiffness(), sp.dampening()};
    }

#ifdef HAS_YAML_CPP
    void spring2D::write(YAML::Emitter &out) const
    {
        joint2D::write(out);
        out << YAML::Key << "Stiffness" << YAML::Value << m_stiffness;
        out << YAML::Key << "Dampening" << YAML::Value << m_dampening;
    }
    YAML::Node spring2D::encode() const
    {
        YAML::Node node = joint2D::encode();
        node["Stiffness"] = m_stiffness;
        node["Dampening"] = m_dampening;
        return node;
    }
    bool spring2D::decode(const YAML::Node &node)
    {
        if (!joint2D::decode(node))
            return false;
        m_stiffness = node["Stiffness"].as<float>();
        m_dampening = node["Dampening"].as<float>();
        return true;
    }
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    Node convert<ppx::spring2D>::encode(const ppx::spring2D &sp)
    {
        return sp.encode();
    }
    bool convert<ppx::spring2D>::decode(const Node &node, ppx::spring2D &sp)
    {
        return sp.decode(node);
    };
}
#endif