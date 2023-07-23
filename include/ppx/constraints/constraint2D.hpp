#ifndef PPX_CONSTRAINT2D_HPP
#define PPX_CONSTRAINT2D_HPP

#include "ppx/entity2D.hpp"

namespace ppx
{
class constraint2D : public kit::identifiable<>, public kit::serializable
{
  public:
    constraint2D(float stiffness = 500.f, float dampening = 30.f);
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

    using entity_gradient = std::pair<const entity2D *, std::array<float, 3>>;
    virtual std::vector<entity_gradient> constraint_gradients() const = 0;
    virtual std::vector<entity_gradient> constraint_derivative_gradients() const = 0;

  private:
    std::vector<ppx::entity2D::const_ptr> m_entities;
    float m_stiffness;
    float m_dampening;
};
// template <std::size_t N> class constraint2D : public constraint_interface2D
// {
//   public:
//     constraint2D() = default;
//     virtual ~constraint2D() = default;

//     constraint2D(const std::array<entity2D::ptr, N> &entities) : m_grad_entities(entities)
//     {
//         copy_to_const_entities(entities);
//     }

//     constraint2D(const std::array<entity2D::ptr, N> &entities, float stiffness, float dampening)
//         : constraint_interface2D(stiffness, dampening), m_grad_entities(entities)
//     {
//         copy_to_const_entities(entities);
//     }

//     const std::array<entity2D::ptr, N> &entities() const
//     {
//         return m_grad_entities;
//     }
//     void entities(const std::array<entity2D::ptr, N> &entities)
//     {
//         m_grad_entities = entities;
//         copy_to_const_entities(entities);
//     }

//     float value() const override
//     {
//         return constraint(m_entities);
//     }
//     virtual bool valid() const override
//     {
//         for (const entity2D::const_ptr &e : m_entities)
//             if (!e)
//                 return false;
//         for (const entity2D::ptr &e : m_grad_entities)
//             if (!e)
//                 return false;
//         return true;
//     }

//   private:
//     std::array<entity2D::const_ptr, N> m_entities;
//     std::array<entity2D::ptr, N> m_grad_entities;

//     void copy_to_const_entities(const std::array<entity2D::ptr, N> &entities)
//     {
//         for (std::size_t i = 0; i < N; i++)
//             m_entities[i] = entities[i];
//     }

//     virtual float constraint(const std::array<entity2D::const_ptr, N> &entities) const = 0;
//     virtual float constraint_derivative(const std::array<entity2D::const_ptr, N> &entities) const = 0;

//     float derivative() const override
//     {
//         return constraint_derivative(m_entities);
//     }
//     std::size_t size() const override
//     {
//         return N;
//     }
//     entity2D &operator[](std::size_t index) const override
//     {
//         return *m_grad_entities[index];
//     }
// };
} // namespace ppx

#endif