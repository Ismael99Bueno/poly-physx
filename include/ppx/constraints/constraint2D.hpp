#ifndef PPX_CONSTRAINT2D_HPP
#define PPX_CONSTRAINT2D_HPP

#include "ppx/body2D.hpp"
#include "kit/interface/nameable.hpp"

namespace ppx
{
class constraint2D : public kit::identifiable<>, public kit::serializable, public kit::nameable
{
  public:
    constraint2D(const char *name, float stiffness = 500.f, float dampening = 30.f);
    virtual ~constraint2D() = default;

    float stiffness() const;
    float dampening() const;

    void stiffness(float stiffness);
    void dampening(float dampening);

#ifdef KIT_USE_YAML_CPP
    virtual YAML::Node encode() const override;
    virtual bool decode(const YAML::Node &node) override;
#endif

    virtual bool valid() const = 0;

    virtual float constraint_value() const = 0;
    virtual float constraint_derivative() const = 0;

    virtual std::size_t size() const = 0;
    virtual const body2D::ptr &body(std::size_t index) const = 0;
    virtual void body(std::size_t index, const body2D::ptr &body) = 0;

    using body_gradient = std::pair<const body2D *, std::array<float, 3>>;
    virtual std::vector<body_gradient> constraint_gradients() const = 0;
    virtual std::vector<body_gradient> constraint_derivative_gradients() const = 0;

  private:
    float m_stiffness;
    float m_dampening;
};
} // namespace ppx

#endif