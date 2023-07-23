#ifndef PPX_CONSTRAINT2D_HPP
#define PPX_CONSTRAINT2D_HPP

#include "ppx/body2D.hpp"

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

    using body_gradient = std::pair<const body2D *, std::array<float, 3>>;
    virtual std::vector<body_gradient> constraint_gradients() const = 0;
    virtual std::vector<body_gradient> constraint_derivative_gradients() const = 0;

  private:
    std::vector<ppx::body2D::const_ptr> m_bodies;
    float m_stiffness;
    float m_dampening;
};
// template <std::size_t N> class constraint2D : public constraint_interface2D
// {
//   public:
//     constraint2D() = default;
//     virtual ~constraint2D() = default;

//     constraint2D(const std::array<body2D::ptr, N> &bodies) : m_grad_bodies(bodies)
//     {
//         copy_to_const_bodies(bodies);
//     }

//     constraint2D(const std::array<body2D::ptr, N> &bodies, float stiffness, float dampening)
//         : constraint_interface2D(stiffness, dampening), m_grad_bodies(bodies)
//     {
//         copy_to_const_bodies(bodies);
//     }

//     const std::array<body2D::ptr, N> &bodies() const
//     {
//         return m_grad_bodies;
//     }
//     void bodies(const std::array<body2D::ptr, N> &bodies)
//     {
//         m_grad_bodies = bodies;
//         copy_to_const_bodies(bodies);
//     }

//     float value() const override
//     {
//         return constraint(m_bodies);
//     }
//     virtual bool valid() const override
//     {
//         for (const body2D::const_ptr &bd : m_bodies)
//             if (!bd)
//                 return false;
//         for (const body2D::ptr &bd : m_grad_bodies)
//             if (!bd)
//                 return false;
//         return true;
//     }

//   private:
//     std::array<body2D::const_ptr, N> m_bodies;
//     std::array<body2D::ptr, N> m_grad_bodies;

//     void copy_to_const_bodies(const std::array<body2D::ptr, N> &bodies)
//     {
//         for (std::size_t i = 0; i < N; i++)
//             m_bodies[i] = bodies[i];
//     }

//     virtual float constraint(const std::array<body2D::const_ptr, N> &bodies) const = 0;
//     virtual float constraint_derivative(const std::array<body2D::const_ptr, N> &bodies) const = 0;

//     float derivative() const override
//     {
//         return constraint_derivative(m_bodies);
//     }
//     std::size_t size() const override
//     {
//         return N;
//     }
//     body2D &operator[](std::size_t index) const override
//     {
//         return *m_grad_bodies[index];
//     }
// };
} // namespace ppx

#endif