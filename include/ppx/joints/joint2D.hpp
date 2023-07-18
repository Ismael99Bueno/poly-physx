#ifndef PPX_JOINT2D_HPP
#define PPX_JOINT2D_HPP

#include "ppx/entity2D.hpp"

namespace ppx
{
class joint2D
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

    void bind(const entity2D::ptr &e1, const entity2D::ptr &e2); // This shit has to be virtual
    virtual bool valid() const;

    const entity2D::ptr &e1() const;
    const entity2D::ptr &e2() const;

    glm::vec2 anchor1() const;
    glm::vec2 anchor2() const;

    void anchor1(const glm::vec2 &anchor1);
    void anchor2(const glm::vec2 &anchor2);

    bool has_anchors() const;

  protected:
    entity2D::ptr m_e1 = nullptr, m_e2 = nullptr;
    glm::vec2 m_anchor1{0.f}, m_anchor2{0.f};
    float m_angle1, m_angle2;
    bool m_has_anchors;

#ifdef YAML_CPP_COMPAT
    virtual void write(YAML::Emitter &out) const;
    virtual YAML::Node encode() const;
    virtual bool decode(const YAML::Node &node);
    friend YAML::Emitter &operator<<(YAML::Emitter &, const joint2D &);
    friend struct YAML::convert<joint2D>;
#endif
};

#ifdef YAML_CPP_COMPAT
YAML::Emitter &operator<<(YAML::Emitter &out, const joint2D &joint);
#endif
} // namespace ppx

#ifdef YAML_CPP_COMPAT
namespace YAML
{
template <> struct convert<ppx::joint2D>
{
    static Node encode(const ppx::joint2D &joint);
    static bool decode(const Node &node, ppx::joint2D &joint);
};
} // namespace YAML
#endif

#endif