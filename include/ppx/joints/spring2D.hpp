#ifndef PPX_SPRING2D_HPP
#define PPX_SPRING2D_HPP

#include "ppx/joints/joint2D.hpp"
#include <utility>

namespace ppx
{
class spring2D : public joint2D, public kit::identifiable, public kit::indexable
{
  public:
    using ptr = kit::track_ptr<spring2D>;
    using const_ptr = kit::const_track_ptr<spring2D>;

    struct specs : joint2D::specs
    {
        float stiffness = 1.f, dampening = 0.f, length = 0.f;
        static specs from_spring(const spring2D &sp);
    };
    spring2D(const entity2D::ptr &e1, const entity2D::ptr &e2, float stiffness = 1.f, float dampening = 0.f,
             float length = 0.f);
    spring2D(const entity2D::ptr &e1, const entity2D::ptr &e2, const glm::vec2 &anchor1, const glm::vec2 &anchor2,
             float stiffness = 1.f, float dampening = 0.f, float length = 0.f);
    spring2D(const specs &spc);

    std::tuple<glm::vec2, float, float> force() const;

    float length() const;
    void length(float length);

    float stiffness() const;
    float dampening() const;

    void stiffness(float stiffness);
    void dampening(float dampening);

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

  private:
    float m_stiffness, m_dampening, m_length;

    std::tuple<glm::vec2, float, float> without_anchors_force() const;
    std::tuple<glm::vec2, float, float> with_anchors_force() const;

#ifdef KIT_USE_YAML_CPP
    void write(YAML::Emitter &out) const override;
    YAML::Node encode() const override;
    bool decode(const YAML::Node &node) override;
    friend struct YAML::convert<spring2D>;
#endif
};

} // namespace ppx

#ifdef KIT_USE_YAML_CPP
namespace YAML
{
template <> struct convert<ppx::spring2D>
{
    static Node encode(const ppx::spring2D &sp);
    static bool decode(const Node &node, ppx::spring2D &sp);
};
} // namespace YAML
#endif

#endif