#pragma once

#include "ppx/internal/worldref.hpp"
#include "ppx/common/specs2D.hpp"
#include "ppx/body/state2D.hpp"
#include "kit/memory/vector_ptr.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/interface/indexable.hpp"
#include <variant>

namespace ppx
{
class world2D;
class collider2D;
class joint2D;
class body2D final : public kit::indexable, public worldref2D, kit::non_copyable
{
  public:
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
    };

    body2D(world2D &world, const specs &spc = {});

    glm::vec2 instant_force{0.f};
    glm::vec2 persistent_force{0.f};

    float instant_torque = 0.f;
    float persistent_torque = 0.f;

    float charge;

    state2D ctr_state;

    const collider2D *operator[](std::size_t index) const;
    collider2D *operator[](std::size_t index);

    collider2D *add(const ppx::specs::collider2D &spc);

    bool remove(std::size_t index);
    bool remove(const collider2D *collider);

    bool contains(const collider2D *collider) const;
    void clear();

    auto begin() const
    {
        return m_colliders.begin();
    }
    auto end() const
    {
        return m_colliders.end();
    }

    auto begin()
    {
        return m_colliders.begin();
    }
    auto end()
    {
        return m_colliders.end();
    }

    bool empty() const;
    std::size_t size() const;

    bool is_dynamic() const;
    bool is_kinematic() const;
    bool is_static() const;

    bool joint_prevents_collision(const body2D *body) const;
    bool attached_to(const body2D *body) const;
    bool attached_to(const joint2D *joint) const;

    btype type() const;
    void type(btype type);

    void begin_density_update();
    void end_density_update();

    void begin_spatial_update();
    void end_spatial_update();

    float kinetic_energy() const;

    glm::vec2 local_centroid_point(const glm::vec2 &gpoint) const;
    glm::vec2 global_centroid_point(const glm::vec2 &lpoint) const;

    glm::vec2 local_position_point(const glm::vec2 &gpoint) const;
    glm::vec2 global_position_point(const glm::vec2 &lpoint) const;

    glm::vec2 local_vector(const glm::vec2 &gvector) const;
    glm::vec2 global_vector(const glm::vec2 &lvector) const;

    void ladd_force_at(const glm::vec2 &force, const glm::vec2 &lpoint);
    void gadd_force_at(const glm::vec2 &force, const glm::vec2 &gpoint);
    void add_force(const glm::vec2 &force);

    const glm::vec2 &force() const;
    float torque() const;

    const properties &props() const;
    const std::vector<joint2D *> &joints() const;

    void translate(const glm::vec2 &dpos);
    void rotate(float dangle);

    const transform2D &centroid_transform() const;
    void centroid_transform(const transform2D &centroid);

    const glm::vec2 &centroid() const;
    const glm::vec2 &lposition() const;
    const glm::vec2 &gposition() const;
    const glm::vec2 &origin() const;
    float rotation() const;

    const glm::vec2 &charge_centroid() const;

    const glm::vec2 &velocity() const;
    glm::vec2 &velocity();
    float angular_velocity() const;
    float &angular_velocity();

    glm::vec2 lvelocity_at_from_centroid(const glm::vec2 &lpoint) const;
    glm::vec2 lvelocity_at_from_position(const glm::vec2 &lpoint) const;
    glm::vec2 gvelocity_at(const glm::vec2 &gpoint) const;
    glm::vec2 velocity_at_centroid_offset(const glm::vec2 &offset) const;
    glm::vec2 velocity_at_position_offset(const glm::vec2 &offset) const;

    void centroid(const glm::vec2 &centroid);
    void gposition(const glm::vec2 &gposition);
    void origin(const glm::vec2 &origin);
    void rotation(float rotation);
    void mass(float mass);

    void velocity(const glm::vec2 &velocity);
    void angular_velocity(float angular_velocity);

    void apply_simulation_force(const glm::vec2 &force);
    void apply_simulation_torque(float torque);

    void full_update();
    void update_colliders();
    void update_centroids();
    void update_inertia();

    bool density_updating() const;
    bool spatial_updating() const;

  private:
    state2D m_state;
    properties m_props;
    glm::vec2 m_charge_centroid;
    glm::vec2 m_force{0.f};
    float m_torque = 0.f;

    std::vector<collider2D *> m_colliders;
    std::vector<joint2D *> m_joints;
    btype m_type;

    bool m_density_update = false;
    bool m_spatial_update = false;

    void reset_simulation_forces();
    void retrieve_data_from_state_variables(const std::vector<float> &vars_buffer);

    void reset_dynamic_properties();

    friend class joint2D;
    friend class collider_manager2D;
    friend class body_manager2D;
};
} // namespace ppx
