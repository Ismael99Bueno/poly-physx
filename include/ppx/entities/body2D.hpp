#pragma once

#include "ppx/entities/collider2D.hpp"
#include "ppx/events/body_events.hpp"
#include "ppx/internal/worldref.hpp"
#include <variant>

namespace ppx
{
class world2D;
class body2D : public kit::identifiable<>, public kit::indexable, public worldref2D
{
  public:
    using ptr = kit::vector_ptr<body2D>;
    using const_ptr = kit::const_vector_ptr<body2D>;

    enum class shape_type
    {
        POLYGON = 0,
        CIRCLE = 1
    };
    struct specs
    {
        glm::vec2 position{0.f};
        glm::vec2 velocity{0.f};
        float rotation = 0.f;
        float angular_velocity = 0.f;
        float mass = 1.f;
        float charge = 1.f;
        kit::dynarray<glm::vec2, PPX_MAX_VERTICES> vertices = polygon::square(5.f);
        float radius = 2.5f;
        bool kinematic = true;
        shape_type shape = shape_type::POLYGON;
        static specs from_body(const body2D &body);
    };

    body2D(world2D &world);
    body2D(world2D &world, const specs &spc);

    glm::vec2 impulse_force{0.f};
    glm::vec2 persistent_force{0.f};

    float impulse_torque = 0.f;
    float persistent_torque = 0.f;

    glm::vec2 velocity;
    float angular_velocity;
    float charge;

    bool kinematic;
    body_events events;

    glm::vec2 constraint_velocity;
    float constraint_angular_velocity;

    const_ptr as_ptr() const;
    ptr as_ptr();

    float kinetic_energy() const;

    void add_force_at(const glm::vec2 &force, const glm::vec2 &at);

    void apply_simulation_force(const glm::vec2 &force);
    void apply_simulation_force_at(const glm::vec2 &force, const glm::vec2 &at);
    void apply_simulation_torque(float torque);

    const glm::vec2 &force() const;
    float torque() const;

    const shape2D &shape() const;
    template <typename T> const T &shape() const;
    template <typename T> const T *shape_if() const;

    void shape(const kit::dynarray<glm::vec2, PPX_MAX_VERTICES> &vertices);
    void shape(float radius);
    void shape(const polygon &poly);
    void shape(const circle &c);

    bool is_polygon() const;
    bool is_circle() const;
    shape_type type() const;

    float mass() const;
    float inv_mass() const;

    float inertia() const;
    float inv_inertia() const;

    float real_mass() const;
    float real_inv_mass() const;

    float real_inertia() const;
    float real_inv_inertia() const;

    void translate(const glm::vec2 &dpos);
    void rotate(float dangle);

    const kit::transform2D<float> &transform() const;

    const glm::vec2 &position() const;
    float rotation() const;

    glm::vec2 velocity_at(const glm::vec2 &at) const;
    glm::vec2 constraint_velocity_at(const glm::vec2 &at) const;

    void position(const glm::vec2 &position);
    void rotation(float rotation);
    void mass(float mass);

    void reset_simulation_forces();
    void retrieve_data_from_state_variables(const std::vector<float> &vars_buffer);

  private:
    std::variant<polygon, circle> m_shape;
    glm::vec2 m_force{0.f};
    float m_torque = 0.f;
    float m_mass;
    float m_inv_mass;
    float m_inertia;
    float m_inv_inertia;

    shape2D &mutable_shape();

    template <typename T> void compute_inertia(const T &shape);
};
} // namespace ppx
