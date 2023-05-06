#include "ppx/pch.hpp"
#include "ppx/rigid_bar2D.hpp"
#include "debug/debug.hpp"

namespace ppx
{
    rigid_bar2D::rigid_bar2D(const entity2D_ptr &e1,
                             const entity2D_ptr &e2,
                             const float stiffness,
                             const float dampening) : constraint2D<2>({e1, e2}, stiffness, dampening),
                                                      m_e1(e1), m_e2(e2),
                                                      m_length(glm::distance(e1->pos(), e2->pos())),
                                                      m_has_anchors(false) {}

    rigid_bar2D::rigid_bar2D(const entity2D_ptr &e1,
                             const entity2D_ptr &e2,
                             const glm::vec2 &anchor1,
                             const glm::vec2 &anchor2,
                             const float stiffness,
                             const float dampening) : constraint2D<2>({e1, e2}, stiffness, dampening),
                                                      m_e1(e1), m_e2(e2),
                                                      m_length(glm::distance(e1->pos() + anchor1,
                                                                             e2->pos() + anchor2)),
                                                      m_angle1(e1->angpos()),
                                                      m_angle2(e2->angpos()),
                                                      m_anchor1(anchor1),
                                                      m_anchor2(anchor2),
                                                      m_has_anchors(true)
    {
    }

    float rigid_bar2D::constraint(const std::array<const_entity2D_ptr, 2> &entities) const
    {
        return m_has_anchors ? with_anchors_constraint(entities) : without_anchors_constraint(entities);
    }

    float rigid_bar2D::constraint_derivative(const std::array<const_entity2D_ptr, 2> &entities) const
    {
        return m_has_anchors ? with_anchors_constraint_derivative(entities) : without_anchors_constraint_derivative(entities);
    }

    float rigid_bar2D::without_anchors_constraint(const std::array<const_entity2D_ptr, 2> &entities) const
    {
        return glm::distance2(m_e1->pos(), m_e2->pos()) - m_length * m_length;
    }
    float rigid_bar2D::without_anchors_constraint_derivative(const std::array<const_entity2D_ptr, 2> &entities) const
    {
        return 2.f * glm::dot(m_e1->pos() - m_e2->pos(), m_e1->vel() - m_e2->vel());
    }

    float rigid_bar2D::with_anchors_constraint(const std::array<const_entity2D_ptr, 2> &entities) const
    {
        const glm::vec2 p1 = anchor1() + m_e1->pos(),
                        p2 = anchor2() + m_e2->pos();

        return glm::distance2(p1, p2) - m_length * m_length;
    }
    float rigid_bar2D::with_anchors_constraint_derivative(const std::array<const_entity2D_ptr, 2> &entities) const
    {
        const glm::vec2 rot_anchor1 = anchor1(),
                        rot_anchor2 = anchor2();

        return 2.f * glm::dot(rot_anchor1 - rot_anchor2 + m_e1->pos() - m_e2->pos(),
                              m_e1->vel_at(rot_anchor1) - m_e2->vel_at(rot_anchor2));
    }

    std::array<float, 3> rigid_bar2D::constraint_grad(entity2D &e) const
    {
        DBG_ASSERT(e == *m_e1 || e == *m_e2, "Passed entity to compute constraint gradient must be equal to some entity of the constraint!\n")
        if (!m_has_anchors)
        {
            const glm::vec2 cg = 2.f * (m_e1->pos() - m_e2->pos());
            if (e == *m_e1)
                return {cg.x, cg.y, 0.f};
            return {-cg.x, -cg.y, 0.f};
        }
        const float a1 = m_e1->angpos() - m_angle1,
                    a2 = m_e2->angpos() - m_angle2;
        const glm::vec2 rot_anchor1 = glm::rotate(m_anchor1, a1),
                        rot_anchor2 = glm::rotate(m_anchor2, a2);
        const glm::vec2 cg = 2.f * (m_e1->pos() + rot_anchor1 - m_e2->pos() - rot_anchor2);
        if (e == *m_e1)
        {
            const float cos = cosf(a1), sin = sinf(a1);
            const float cga = -cg.x * (m_anchor1.x * sin + m_anchor1.y * cos) +
                              cg.y * (m_anchor1.x * cos - m_anchor1.y * sin);
            return {cg.x, cg.y, cga};
        }
        const float cos = cosf(a2), sin = sinf(a2);
        const float cga = cg.x * (m_anchor2.x * sin + m_anchor2.y * cos) +
                          cg.y * (-m_anchor2.x * cos + m_anchor2.y * sin);
        return {-cg.x, -cg.y, cga};
    }
    std::array<float, 3> rigid_bar2D::constraint_grad_derivative(entity2D &e) const
    {
        DBG_ASSERT(e == *m_e1 || e == *m_e2, "Passed entity to compute constraint gradient must be equal to some entity of the constraint!\n")
        if (!m_has_anchors)
        {
            const glm::vec2 cgd = 2.f * (m_e1->vel() - m_e2->vel());
            if (e == *m_e1)
                return {cgd.x, cgd.y, 0.f};
            return {-cgd.x, -cgd.y, 0.f};
        }
        const glm::vec2 rot_anchor1 = anchor1(),
                        rot_anchor2 = anchor2();
        const glm::vec2 cgd = 2.f * (m_e1->vel_at(rot_anchor1) - m_e2->vel_at(rot_anchor2));
        if (e == *m_e1)
        {
            const float cgda = -cgd.x * rot_anchor1.x - cgd.y * rot_anchor1.y;
            return {cgd.x, cgd.y, cgda};
        }
        const float cgda = cgd.x * rot_anchor2.x + cgd.y * rot_anchor2.y;
        return {-cgd.x, -cgd.y, cgda};
    }

    float rigid_bar2D::length() const { return m_length; }
    void rigid_bar2D::length(const float length) { m_length = length; }

    const entity2D_ptr &rigid_bar2D::e1() const { return m_e1; }
    const entity2D_ptr &rigid_bar2D::e2() const { return m_e2; }

    glm::vec2 rigid_bar2D::anchor1() const { return glm::rotate(m_anchor1, m_e1->angpos() - m_angle1); }
    glm::vec2 rigid_bar2D::anchor2() const { return glm::rotate(m_anchor2, m_e2->angpos() - m_angle2); }

    void rigid_bar2D::anchor1(const glm::vec2 &anchor1)
    {
        m_anchor1 = anchor1;
        m_angle1 = m_e1->angpos();
        m_has_anchors = true;
    }
    void rigid_bar2D::anchor2(const glm::vec2 &anchor2)
    {
        m_anchor2 = anchor2;
        m_angle2 = m_e2->angpos();
        m_has_anchors = true;
    }

    bool rigid_bar2D::has_anchors() const { return m_has_anchors; }
    bool rigid_bar2D::try_validate()
    {
        return constraint2D::try_validate() && m_e1.try_validate() && m_e2.try_validate();
    }

#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const rigid_bar2D &rb)
    {
        out << YAML::BeginMap;
        out << YAML::Key << "ID1" << YAML::Value << rb.e1().id();
        out << YAML::Key << "ID2" << YAML::Value << rb.e2().id();
        out << YAML::Key << "Index1" << YAML::Value << rb.e1().index();
        out << YAML::Key << "Index2" << YAML::Value << rb.e2().index();
        if (rb.has_anchors())
        {
            out << YAML::Key << "Anchor1" << YAML::Value << rb.anchor1();
            out << YAML::Key << "Anchor2" << YAML::Value << rb.anchor2();
        }
        out << YAML::Key << "Stiffness" << YAML::Value << rb.stiffness();
        out << YAML::Key << "Dampening" << YAML::Value << rb.dampening();
        out << YAML::Key << "length" << YAML::Value << rb.length();
        out << YAML::EndMap;
        return out;
    }
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    Node convert<ppx::rigid_bar2D>::encode(const ppx::rigid_bar2D &rb)
    {
        Node node;
        node["ID1"] = rb.e1().id();
        node["ID2"] = rb.e2().id();
        node["Index1"] = rb.e1().index();
        node["Index2"] = rb.e2().index();
        if (rb.has_anchors())
        {
            node["Anchor1"] = rb.anchor1();
            node["Anchor2"] = rb.anchor2();
        }
        node["Stiffness"] = rb.stiffness();
        node["Dampening"] = rb.dampening();
        node["length"] = rb.length();
        return node;
    }
    bool convert<ppx::rigid_bar2D>::decode(const Node &node, ppx::rigid_bar2D &rb)
    {
        if (!node.IsMap() || (node.size() != 7 && node.size() != 9))
            return false;

        rb.stiffness(node["Stiffness"].as<float>());
        rb.dampening(node["Dampening"].as<float>());
        rb.length(node["length"].as<float>());

        return true;
    };
}
#endif