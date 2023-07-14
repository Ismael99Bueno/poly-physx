#ifndef PPX_JOINT2D_HPP
#define PPX_JOINT2D_HPP
#include "ppx/internal/core.hpp"

#include "ppx/entity2D_ptr.hpp"

namespace ppx
{
class joint2D
{
  public:
    struct specs
    {
        entity2D_ptr e1 = nullptr, e2 = nullptr;
        glm::vec2 anchor1{0.f}, anchor2{0.f};
        float length = 0.f;
        bool has_anchors = true;
        static specs from_joint(const joint2D &joint);
    };
    joint2D(const entity2D_ptr &e1, const entity2D_ptr &e2, float length);
    joint2D(const entity2D_ptr &e1, const entity2D_ptr &e2, const glm::vec2 &anchor1, const glm::vec2 &anchor2,
            float length);
    joint2D(const specs &spc);
    virtual ~joint2D() = default;

    void bind(const entity2D_ptr &e1, const entity2D_ptr &e2);
    virtual bool validate();

    float length() const;
    void length(float length);

    const entity2D_ptr &e1() const;
    const entity2D_ptr &e2() const;

    glm::vec2 anchor1() const;
    glm::vec2 anchor2() const;

    void anchor1(const glm::vec2 &anchor1);
    void anchor2(const glm::vec2 &anchor2);

    bool has_anchors() const;

  protected:
    entity2D_ptr m_e1 = nullptr, m_e2 = nullptr;
    glm::vec2 m_anchor1{0.f}, m_anchor2{0.f};
    float m_angle1, m_angle2, m_length;
    bool m_has_anchors;

#ifdef HAS_YAML_CPP
    virtual void write(YAML::Emitter &out) const;
    virtual YAML::Node encode() const;
    virtual bool decode(const YAML::Node &node);
    friend YAML::Emitter &operator<<(YAML::Emitter &, const joint2D &);
    friend struct YAML::convert<joint2D>;
#endif
};

#ifdef HAS_YAML_CPP
YAML::Emitter &operator<<(YAML::Emitter &out, const joint2D &joint);
#endif
} // namespace ppx

#ifdef HAS_YAML_CPP
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