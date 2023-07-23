#ifndef PPX_JOINT2D_HPP
#define PPX_JOINT2D_HPP

#include "ppx/entity2D.hpp"
#include "kit/interface/serialization.hpp"

namespace ppx
{
class joint2D : public kit::serializable
{
  public:
    struct specs
    {
        entity2D::ptr e1 = nullptr, e2 = nullptr;
        glm::vec2 anchor1{0.f}, anchor2{0.f};
        bool has_anchors = true;
        static specs from_joint(const joint2D &joint);
    };

    joint2D(const entity2D::ptr &e1, const entity2D::ptr &e2);
    joint2D(const entity2D::ptr &e1, const entity2D::ptr &e2, const glm::vec2 &anchor1, const glm::vec2 &anchor2);
    joint2D(const specs &spc);
    virtual ~joint2D() = default;

    void bind(const entity2D::ptr &e1, const entity2D::ptr &e2);
    virtual bool valid() const;

    const entity2D::ptr &e1() const;
    const entity2D::ptr &e2() const;

    glm::vec2 anchor1() const;
    glm::vec2 anchor2() const;

    void anchor1(const glm::vec2 &anchor1);
    void anchor2(const glm::vec2 &anchor2);

    bool has_anchors() const;

#ifdef KIT_USE_YAML_CPP
    virtual YAML::Node encode() const override;
    virtual bool decode(const YAML::Node &node) override;
#endif

  protected:
    entity2D::ptr m_e1 = nullptr, m_e2 = nullptr;
    glm::vec2 m_anchor1{0.f}, m_anchor2{0.f};
    float m_angle1, m_angle2;
    bool m_has_anchors;
};
} // namespace ppx

#endif