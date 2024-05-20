#include "ppx/internal/pch.hpp"
#include "ppx/collision/resolution/spring_driven_resolution2D.hpp"

#include "kit/multithreading/mt_for_each.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
spring_driven_resolution2D::spring_driven_resolution2D(world2D &world, const float rigidity, const float normal_damping,
                                                       const float tangent_damping)
    : collision_resolution2D(world), rigidity(rigidity), normal_damping(normal_damping),
      tangent_damping(tangent_damping)
{
}
void spring_driven_resolution2D::solve_collisions(const collision_detection2D::collision_map &collisions)
{
    KIT_ASSERT_ERROR(rigidity >= 0.f, "Rigidity must be non-negative: {0}", rigidity)
    KIT_ASSERT_ERROR(tangent_damping >= 0.f, "Tangent damping must be non-negative: {0}", tangent_damping)
    KIT_ASSERT_ERROR(normal_damping >= 0.f, "Normal damping must be non-negative: {0}", normal_damping)

    KIT_PERF_SCOPE("Spring driven solve")
    for (const auto &colis : collisions)
        if (colis.second.collided && colis.second.enabled)
            solve_and_apply_collision_forces(colis.second);
}

std::tuple<glm::vec2, float, float> spring_driven_resolution2D::compute_collision_forces(
    const collision2D &colis, std::size_t manifold_index) const
{
    const body2D *body1 = colis.collider1->body();
    const body2D *body2 = colis.collider2->body();

    const glm::vec2 &touch1 = colis.manifold[manifold_index].point;
    const glm::vec2 touch2 = colis.manifold[manifold_index].point - colis.mtv;

    const glm::vec2 offset1 = touch1 - body1->centroid();
    const glm::vec2 offset2 = touch2 - body2->centroid();

    const glm::vec2 relvel =
        (body2->velocity_at_centroid_offset(offset2) - body1->velocity_at_centroid_offset(offset1));

    const glm::vec2 normal_dir = glm::normalize(colis.mtv);

    const glm::vec2 normal_vel = glm::dot(normal_dir, relvel) * normal_dir;
    const glm::vec2 tangent_vel = relvel - normal_vel;

    const glm::vec2 force = -colis.mtv * rigidity + normal_vel * normal_damping + tangent_vel * tangent_damping;
    const float torque1 = kit::cross2D(offset1, force), torque2 = kit::cross2D(force, offset2);
    return {force, torque1, torque2};
}

void spring_driven_resolution2D::solve_and_apply_collision_forces(const collision2D &colis) const
{
    KIT_PERF_FUNCTION()

    glm::vec2 force{0.f};
    float torque1 = 0.f;
    float torque2 = 0.f;
    for (std::size_t i = 0; i < colis.manifold.size(); i++)
    {
        const auto [f, t1, t2] = compute_collision_forces(colis, i);
        force += f;
        torque1 += t1;
        torque2 += t2;
    }
    force /= colis.manifold.size();
    torque1 /= colis.manifold.size();
    torque2 /= colis.manifold.size();

    body2D *body1 = colis.collider1->body();
    body2D *body2 = colis.collider2->body();

    body1->apply_simulation_force(force);
    body1->apply_simulation_torque(torque1);

    body2->apply_simulation_force(-force);
    body2->apply_simulation_torque(torque2);
}
} // namespace ppx