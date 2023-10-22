#ifndef PPX_BODY2D_HPP
#define PPX_BODY2D_HPP

#include "geo/aabb2D.hpp"
#include "geo/polygon.hpp"
#include "geo/circle.hpp"
#include "rk/state.hpp"
#include "ppx/events/body_events.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/interface/indexable.hpp"
#include "kit/memory/track_ptr.hpp"
#include <variant>

namespace ppx
{
class body2D : public kit::identifiable<>, public kit::indexable
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
        glm::vec2 position{0.f}, velocity{0.f};
        float rotation = 0.f;
        float angular_velocity = 0.f;
        float mass = 1.f;
        float charge = 1.f;
        std::vector<glm::vec2> vertices = geo::polygon::box(5.f);
        float radius = 2.5f;
        bool kinematic = true;
        shape_type shape = shape_type::POLYGON;
        static specs from_body(const body2D &body);
    };

#ifdef KIT_USE_YAML_CPP
    class serializer : public kit::serializer<body2D>
    {
      public:
        YAML::Node encode(const body2D &world) const override;
        bool decode(const YAML::Node &node, body2D &world) const override;
    };
#endif

    body2D(const std::vector<glm::vec2> &vertices, const glm::vec2 &position = glm::vec2(0.f),
           const glm::vec2 &velocity = glm::vec2(0.f), float rotation = 0.f, float angular_velocity = 0.f,
           float mass = 1.f, float charge = 1.f, bool kinematic = true);
    body2D(float radius, const glm::vec2 &position = glm::vec2(0.f), const glm::vec2 &velocity = glm::vec2(0.f),
           float rotation = 0.f, float angular_velocity = 0.f, float mass = 1.f, float charge = 1.f,
           bool kinematic = true);
    body2D(const glm::vec2 &position = glm::vec2(0.f), const glm::vec2 &velocity = glm::vec2(0.f), float rotation = 0.f,
           float angular_velocity = 0.f, float mass = 1.f, float charge = 1.f, bool kinematic = true);
    explicit body2D(const specs &spc);

    glm::vec2 persistent_force{0.f};
    float persistent_torque{0.f};
    bool kinematic;
    body_events events;

    float kinetic_energy() const;

    void add_force(const glm::vec2 &force);
    void add_torque(float torque);

    void apply_simulation_force(const glm::vec2 &force);
    void apply_simulation_torque(float torque);

    const glm::vec2 &force() const;
    const glm::vec2 &added_force() const;

    float torque() const;
    float added_torque() const;

    const geo::shape2D &shape() const;
    template <typename T> const T &shape() const;
    template <typename T> const T *shape_if() const;

    void shape(const std::vector<glm::vec2> &vertices);
    void shape(float radius);
    void shape(const geo::polygon &poly);
    void shape(const geo::circle &c);

    shape_type type() const;

    float effective_mass() const;
    float effective_inverse_mass() const;

    float effective_inertia() const;
    float effective_inverse_inertia() const;

    float real_mass() const;
    float real_inverse_mass() const;

    float real_inertia() const;
    float real_inverse_inertia() const;

    void translate(const glm::vec2 &dpos);
    void rotate(float dangle);

    void boost(const glm::vec2 &dvel);
    void spin(float dangvel);

    const kit::transform2D &transform() const;

    const glm::vec2 &position() const;
    const glm::vec2 &velocity() const;
    glm::vec2 velocity_at(const glm::vec2 &at) const;

    float rotation() const;
    float angular_velocity() const;
    float charge() const;

    void position(const glm::vec2 &position);
    void velocity(const glm::vec2 &velocity);
    void rotation(float rotation);
    void angular_velocity(float angular_velocity);
    void mass(float mass);
    void charge(float charge);

  private:
    std::variant<geo::polygon, geo::circle> m_shape;
    glm::vec2 m_velocity{0.f};
    glm::vec2 m_force{0.f};
    glm::vec2 m_added_force{0.f};
    float m_angular_velocity;
    float m_torque = 0.f;
    float m_added_torque = 0.f;
    float m_mass;
    float m_inv_mass;
    float m_inertia;
    float m_inv_inertia;
    float m_charge;

    geo::shape2D &mutable_shape();
    void retrieve_data_from_state_variables(const std::vector<float> &vars_buffer);

    void reset_added_forces();
    void reset_simulation_forces();

    template <typename T> void compute_inertia(const T &shape);

    friend class world2D;
};
} // namespace ppx
#endif