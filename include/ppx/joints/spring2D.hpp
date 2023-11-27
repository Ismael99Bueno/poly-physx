#ifndef PPX_SPRING2D_HPP
#define PPX_SPRING2D_HPP

#include "ppx/joints/joint_proxy2D.hpp"

namespace ppx
{
class spring2D : public kit::identifiable<>, public kit::indexable
{
  public:
    using ptr = kit::vector_ptr<spring2D>;
    using const_ptr = kit::const_vector_ptr<spring2D>;

#ifdef KIT_USE_YAML_CPP
    class serializer : public kit::serializer<spring2D>
    {
      public:
        YAML::Node encode(const spring2D &world) const override;
        bool decode(const YAML::Node &node, spring2D &world) const override;
    };
#endif

    struct specs
    {
        joint_proxy2D::specs joint;
        float stiffness = 1.f, dampening = 0.f, length = 0.f;
        static specs from_spring(const spring2D &sp);
    };

    spring2D(float stiffness = 1.f, float dampening = 0.f, float length = 0.f);
    spring2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1 = glm::vec2(0.f),
             const glm::vec2 &anchor2 = glm::vec2(0.f), float stiffness = 1.f, float dampening = 0.f,
             float length = 0.f);
    spring2D(const specs &spc);

    joint_proxy2D joint;
    float stiffness;
    float dampening;
    float length;

    glm::vec4 force() const;

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    void apply_force_to_bodies();

  private:
    world2D *m_world = nullptr;
    friend class spring_manager2D;
};

} // namespace ppx

#endif