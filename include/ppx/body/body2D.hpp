#pragma once

#include "ppx/internal/worldref.hpp"
#include "ppx/common/specs2D.hpp"
#include "ppx/body/state2D.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/interface/non_copyable.hpp"
#include <variant>

namespace ppx
{
class world2D;
class collider2D;
class joint2D;
class contact2D;
class island2D;
class body2D final : public worldref2D, kit::non_copyable
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

    struct metadata
    {
        std::size_t index;
        island2D *island = nullptr;
        bool island_flag = false;

        std::vector<joint2D *> joints;
        std::vector<contact2D *> contacts;
        void remove_joint(const joint2D *joint);
        void remove_contact(const contact2D *contact);
    } meta; // easy read & write data

    const collider2D *operator[](std::size_t index) const;
    collider2D *operator[](std::size_t index);

    collider2D *add(const ppx::specs::collider2D &spc);

    bool remove(std::size_t index);
    bool remove(collider2D *collider);

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

    void awake(bool even_if_non_dynamic = false);
    bool asleep() const;

    bool is_dynamic() const;
    bool is_kinematic() const;
    bool is_static() const;

    bool joint_prevents_collision(const body2D *body) const;
    bool attached_to(const body2D *body) const;
    bool attached_to(const joint2D *joint) const;

    btype type() const;
    void type(btype type);

    void begin_density_update();
    void end_density_update(bool update_bbox = true);

    void begin_spatial_update();
    void end_spatial_update(bool update_bbox = true);

    float kinetic_energy() const;
    const state2D &state() const;

    void ladd_force_at(const glm::vec2 &force, const glm::vec2 &lpoint);
    void gadd_force_at(const glm::vec2 &force, const glm::vec2 &gpoint);

    void add_force(const glm::vec2 &force);

    const glm::vec2 &instant_force() const;
    const glm::vec2 &persistent_force() const;

    float instant_torque() const;
    float persistent_torque() const;

    void instant_force(const glm::vec2 &force);
    void persistent_force(const glm::vec2 &force);
    void instant_torque(float torque);
    void persistent_torque(float torque);

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
    float angular_velocity() const;

    float mass() const;
    float inv_mass() const;
    void mass(float mass);

    float inertia() const;
    float inv_inertia() const;

    float charge() const;
    void charge(float charge);

    void centroid(const glm::vec2 &centroid);
    void gposition(const glm::vec2 &gposition);
    void origin(const glm::vec2 &origin);
    void rotation(float rotation);

    void velocity(const glm::vec2 &velocity);
    void angular_velocity(float angular_velocity);

    void full_update();
    void update_colliders(bool update_bbox = true);
    void update_centroids();
    void update_inertia();

    bool density_updating() const;
    bool spatial_updating() const;

    const std::vector<joint2D *> &joints() const;
    const std::vector<contact2D *> &contacts() const;

    bool checksum() const;

  private:
    state2D m_state;
    glm::vec2 m_gposition; // not relevant for the state

    glm::vec2 m_instant_force{0.f};
    glm::vec2 m_persistent_force{0.f};

    float m_instant_torque = 0.f;
    float m_persistent_torque = 0.f;

    std::vector<collider2D *> m_colliders;

    bool m_density_update = false;
    bool m_spatial_update = false;
    bool m_awake_allowed = true;

    void retrieve_data_from_state(const state2D &state, bool update_bbox);
    void stop_all_motion();

    friend class collider_manager2D;
    friend class body_manager2D;
};
} // namespace ppx
