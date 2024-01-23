#pragma once

#include "ppx/entities/body2D.hpp"

namespace ppx
{
class world2D;
class joint_proxy2D
{
  public:
    struct specs
    {
        body2D::ptr body1 = nullptr, body2 = nullptr;
        glm::vec2 anchor1{0.f}, anchor2{0.f};
    };

    joint_proxy2D() = default;
    joint_proxy2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1 = glm::vec2(0.f),
                  const glm::vec2 &anchor2 = glm::vec2(0.f));
    joint_proxy2D(const specs &spc);
    virtual ~joint_proxy2D() = default;

    std::tuple<glm::vec2, glm::vec2, glm::vec2> compute_anchors_and_direction() const;
    bool valid() const;

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

  private:
    body2D::ptr m_body1 = nullptr, m_body2 = nullptr;
    glm::vec2 m_anchor1{0.f}, m_anchor2{0.f};
    float m_angle1 = 0.f, m_angle2 = 0.f;
};
} // namespace ppx
