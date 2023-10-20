#ifndef PPX_JOINT2D_HPP
#define PPX_JOINT2D_HPP

#include "ppx/body2D.hpp"
#include "kit/interface/serialization.hpp"

namespace ppx
{
class joint2D : public kit::serializable
{
  public:
    struct specs
    {
        body2D::ptr body1 = nullptr, body2 = nullptr;
        glm::vec2 anchor1{0.f}, anchor2{0.f};
    };

    joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1 = glm::vec2(0.f),
            const glm::vec2 &anchor2 = glm::vec2(0.f));
    joint2D(const specs &spc);
    virtual ~joint2D() = default;

    virtual bool valid() const;

    const body2D::ptr &body1() const;
    const body2D::ptr &body2() const;

    void body1(const body2D::ptr &body1);
    void body2(const body2D::ptr &body2);

    glm::vec2 rotated_anchor1() const;
    glm::vec2 rotated_anchor2() const;

    const glm::vec2 &anchor1() const;
    const glm::vec2 &anchor2() const;

    void anchor1(const glm::vec2 &anchor1);
    void anchor2(const glm::vec2 &anchor2);

#ifdef KIT_USE_YAML_CPP
    virtual YAML::Node encode() const override;
    virtual bool decode(const YAML::Node &node) override;
#endif

  protected:
    body2D::ptr m_body1 = nullptr, m_body2 = nullptr;
    glm::vec2 m_anchor1{0.f}, m_anchor2{0.f};
    float m_angle1, m_angle2;
};
} // namespace ppx

#endif