#ifndef PPX_CONSTRAINT_INTERFACE2D_HPP
#define PPX_CONSTRAINT_INTERFACE2D_HPP

#include "ppx/entity2D.hpp"
#include <vector>
#include <array>
#include <functional>

namespace ppx
{
class constraint_interface2D : public kit::identifiable<>, public kit::serializable
{
  public:
    constraint_interface2D(float stiffness = 500.f, float dampening = 30.f);
    virtual ~constraint_interface2D() = default;

    virtual float value() const = 0;
    float stiffness() const;
    float dampening() const;

    void stiffness(float stiffness);
    void dampening(float dampening);

    virtual bool valid() const = 0;

#ifdef KIT_USE_YAML_CPP
    virtual YAML::Node encode() const override;
    virtual bool decode(const YAML::Node &node) override;
#endif

  private:
    float m_stiffness, m_dampening;

    virtual std::array<float, 3> constraint_grad(entity2D &e) const;
    virtual std::array<float, 3> constraint_grad_derivative(entity2D &e) const;

    using constraint_fun = std::function<float(const constraint_interface2D &)>;
    std::array<float, 3> gradient(entity2D &e, const constraint_fun &constraint) const;

    virtual float derivative() const = 0;
    virtual std::size_t size() const = 0;
    virtual entity2D &operator[](std::size_t index) const = 0;

    friend class compeller2D;
};
} // namespace ppx

#endif