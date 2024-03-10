#pragma once

#include "kit/interface/identifiable.hpp"
#include "kit/interface/indexable.hpp"
#include "kit/memory/vector_ptr.hpp"
#include "kit/utility/type_constraints.hpp"
#include "ppx/internal/worldref.hpp"
#include "ppx/entities/specs2D.hpp"
#include <variant>

namespace ppx
{
class world2D;
class collider2D;
class body2D : public kit::identifiable<>, public kit::indexable, public worldref2D
{
  public:
    using ptr = kit::vector_ptr<body2D>;
    using const_ptr = kit::const_vector_ptr<body2D>;
    using specs = specs::body2D;
    using btype = specs::btype;

    struct properties
    {
        struct data
        {
            float mass;
            float inv_mass;
            float inertia;
            float inv_inertia;
        };
        data dynamic;
        data nondynamic;
        float charge;
    };

    body2D(world2D &world, const specs &spc = {});

    glm::vec2 impulse_force{0.f};
    glm::vec2 persistent_force{0.f};

    float impulse_torque = 0.f;
    float persistent_torque = 0.f;

    glm::vec2 velocity;
    float angular_velocity;
    float charge;

    struct
    {
        glm::vec2 velocity{0.f};
        float angular_velocity = 0.f;
        glm::vec2 velocity_at_centroid_offset(const glm::vec2 &offset) const
        {
            return velocity + angular_velocity * glm::vec2(-offset.y, offset.x);
        } // v + cross(w, at)
    } ctr_proxy;

    const collider2D &operator[](std::size_t index) const;
    collider2D &operator[](std::size_t index);

    const collider2D *operator[](kit::uuid id) const;
    collider2D *operator[](kit::uuid id);

    collider2D &add(const ppx::specs::collider2D &spc);

    bool remove(std::size_t index);
    bool remove(kit::uuid id);
    bool remove(const collider2D &collider);
    void clear();

    std::vector<collider2D>::const_iterator begin() const;
    std::vector<collider2D>::const_iterator end() const;

    std::vector<collider2D>::iterator begin();
    std::vector<collider2D>::iterator end();

    bool empty() const;
    std::size_t size() const;

    bool is_dynamic() const;
    bool is_kinematic() const;
    bool is_static() const;

    btype type() const;
    void type(btype type);

    void begin_density_update();
    void end_density_update();

    void begin_spatial_update();
    void end_spatial_update();

    const_ptr as_ptr() const;
    ptr as_ptr();

    float kinetic_energy() const;

    glm::vec2 local_centroid_point(const glm::vec2 &gpoint) const;
    glm::vec2 global_centroid_point(const glm::vec2 &lpoint) const;

    glm::vec2 local_position_point(const glm::vec2 &gpoint) const;
    glm::vec2 global_position_point(const glm::vec2 &lpoint) const;

    glm::vec2 local_vector(const glm::vec2 &gvector) const;
    glm::vec2 global_vector(const glm::vec2 &lvector) const;

    void ladd_force_at(const glm::vec2 &force, const glm::vec2 &lpoint);
    void gadd_force_at(const glm::vec2 &force, const glm::vec2 &gpoint);

    const glm::vec2 &force() const;
    float torque() const;

    const properties &props() const;

    void translate(const glm::vec2 &dpos);
    void rotate(float dangle);

    const kit::transform2D<float> &centroid_transform() const;
    void centroid_transform(const kit::transform2D<float> &centroid);

    const glm::vec2 &centroid() const;
    const glm::vec2 &lposition() const;
    const glm::vec2 &gposition() const;
    const glm::vec2 &origin() const;
    float rotation() const;

    const glm::vec2 &charge_centroid() const;

    glm::vec2 lvelocity_at_from_centroid(const glm::vec2 &lpoint) const;
    glm::vec2 lvelocity_at_from_position(const glm::vec2 &lpoint) const;
    glm::vec2 gvelocity_at(const glm::vec2 &gpoint) const;
    glm::vec2 gvelocity_at_centroid_offset(const glm::vec2 &offset) const;
    glm::vec2 gvelocity_at_position_offset(const glm::vec2 &offset) const;

    void centroid(const glm::vec2 &centroid);
    void gposition(const glm::vec2 &gposition);
    void origin(const glm::vec2 &origin);
    void rotation(float rotation);
    void mass(float mass);

  private:
    kit::transform2D<float> m_centroid;
    glm::vec2 m_gposition;
    glm::vec2 m_lposition;
    properties m_props;
    glm::vec2 m_charge_centroid;
    glm::vec2 m_force{0.f};
    float m_torque = 0.f;

    std::size_t m_start = 0;
    std::size_t m_size = 0;
    btype m_type;

    bool m_density_update = false;
    bool m_spatial_update = false;

    void update_colliders();
    void update_centroids();
    void update_inertia();

    void reset_simulation_forces();
    void retrieve_data_from_state_variables(const std::vector<float> &vars_buffer);

    void apply_simulation_force(const glm::vec2 &force);
    void apply_simulation_torque(float torque);

    void reset_dynamic_properties();

    friend class collider2D;
    friend class spring2D;
    friend class behaviour2D;
    friend class constraint2D;
    friend class body_manager2D;
    friend class collider_manager2D;
    friend class joint_constraint2D;
    friend class spring_driven_resolution2D;
};
} // namespace ppx
