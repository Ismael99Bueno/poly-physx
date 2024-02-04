#include "ppx/internal/pch.hpp"
#include "ppx/entities/body2D.hpp"
#include "ppx/entities/collider2D.hpp"
#include "ppx/world2D.hpp"
#include "ppx/behaviours/force2D.hpp"
#include "ppx/behaviours/interaction2D.hpp"

namespace ppx
{
body2D::body2D(world2D &world, const body2D::specs &spc)
    : kit::identifiable<>(kit::uuid::random()), worldref2D(world), velocity(spc.velocity),
      angular_velocity(spc.angular_velocity), charge(spc.charge), type(spc.type),
      m_transform(kit::transform2D<float>::builder().position(spc.position).rotation(spc.rotation).build()),
      m_charge_centroid(spc.position)
{
    m_props.position = m_transform.position;
    m_props.dynamic.mass = is_dynamic() ? spc.mass : FLT_MAX;
    m_props.nondynamic.mass = spc.mass;

    begin_density_update();
    for (const auto &collider_spc : spc.colliders)
        add(collider_spc);
    end_density_update();
}

const collider2D &body2D::operator[](const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_size, "Index exceeds array bounds - index: {0}, size: {1}", index, m_size);
    return world.colliders[m_start + index];
}
collider2D &body2D::operator[](const std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_size, "Index exceeds array bounds - index: {0}, size: {1}", index, m_size);
    return world.colliders[m_start + index];
}

const collider2D *body2D::operator[](const kit::uuid id) const
{
    return world.colliders[id];
}
collider2D *body2D::operator[](const kit::uuid id)
{
    return world.colliders[id];
}

collider2D &body2D::add(const ppx::specs::collider2D &spc)
{
    return world.colliders.add(as_ptr(), spc);
}

bool body2D::remove(const std::size_t index)
{
    if (index >= m_size)
        return false;

    return world.colliders.remove(m_start + index);
}
bool body2D::remove(const kit::uuid id)
{
    std::size_t index = SIZE_MAX;
    for (const collider2D &collider : *this)
        if (collider.id == id)
        {
            index = collider.index;
            break;
        }
    return world.colliders.remove(index);
}
bool body2D::remove(const collider2D &collider)
{
    return remove(collider.index);
}
void body2D::clear()
{
    for (std::size_t i = m_size - 1; i < m_size; i--)
        remove(i);
}

std::vector<collider2D>::const_iterator body2D::begin() const
{
    return world.colliders.begin() + m_start;
}
std::vector<collider2D>::const_iterator body2D::end() const
{
    return world.colliders.begin() + m_start + m_size;
}

std::vector<collider2D>::iterator body2D::begin()
{
    return world.colliders.begin() + m_start;
}
std::vector<collider2D>::iterator body2D::end()
{
    return world.colliders.begin() + m_start + m_size;
}

bool body2D::empty() const
{
    return m_size == 0;
}
std::size_t body2D::size() const
{
    return m_size;
}

const body2D::properties &body2D::props() const
{
    return m_props;
}

void body2D::begin_density_update()
{
    KIT_ASSERT_ERROR(!m_density_update, "Cannot begin update while already updating");
    begin_spatial_update();
    m_density_update = true;
}
void body2D::end_density_update()
{
    m_density_update = false;
    update_centroids();
    update_inertia();
    end_spatial_update();
}

void body2D::begin_spatial_update()
{
    KIT_ASSERT_ERROR(!m_spatial_update, "Cannot begin update while already updating");
    m_spatial_update = true;
}
void body2D::end_spatial_update()
{
    m_spatial_update = false;
    update_colliders();
}

void body2D::retrieve_data_from_state_variables(const std::vector<float> &vars_buffer)
{
    const std::size_t idx = 6 * index;

    begin_spatial_update();
    position({vars_buffer[idx + 0], vars_buffer[idx + 1]});
    rotation(vars_buffer[idx + 2]);
    end_spatial_update();

    velocity = {vars_buffer[idx + 3], vars_buffer[idx + 4]};
    angular_velocity = vars_buffer[idx + 5];
}

void body2D::update_colliders()
{
    if (m_spatial_update)
        return;
    for (collider2D &collider : *this)
    {
        shape2D &shape = collider.mutable_shape();
        KIT_ASSERT_ERROR(!shape.updating(), "Cannot update collider while it is already being updated")
        shape.update();
    }
}
void body2D::update_centroids()
{
    if (m_density_update)
        return;
    glm::vec2 centroid{0.f};
    float artificial_mass = 0.f;

    glm::vec2 charge_centroid{0.f};
    float artificial_charge = 0.f;

    for (const collider2D &collider : *this)
    {
        const shape2D &shape = collider.shape();

        const float cmass = collider.density() * shape.area();
        centroid += cmass * shape.centroid();
        artificial_mass += cmass;

        const float ccharge = collider.charge_density() * shape.area();
        charge_centroid += ccharge * shape.centroid();
        artificial_charge += ccharge;
    }
    m_charge_centroid = charge_centroid / artificial_charge;

    const glm::vec2 diff = centroid / artificial_mass - m_transform.position;
    m_transform.position = centroid / artificial_mass;
    for (collider2D &collider : *this)
        collider.mutable_shape().translate(diff);
}
void body2D::update_inertia()
{
    if (m_density_update)
        return;
    m_props.nondynamic.inertia = 0.f;
    float artificial_mass = 0.f;
    for (const collider2D &collider : *this)
    {
        const shape2D &shape = collider.shape();
        const float cmass = collider.density() * shape.area();
        const float dist2 = glm::length2(shape.centroid() - m_transform.position);

        m_props.nondynamic.inertia += cmass * (dist2 + shape.inertia());
        artificial_mass += cmass;
    }
    m_props.nondynamic.inertia *= m_props.nondynamic.mass / artificial_mass;
    if (is_dynamic())
        m_props.dynamic.inertia = m_props.nondynamic.inertia;
}

void body2D::reset_simulation_forces()
{
    m_force = glm::vec2(0.f);
    m_torque = 0.f;
}

bool body2D::is_dynamic() const
{
    return type == btype::DYNAMIC;
}
bool body2D::is_kinematic() const
{
    return type == btype::KINEMATIC;
}
bool body2D::is_static() const
{
    return type == btype::STATIC;
}

body2D::const_ptr body2D::as_ptr() const
{
    return world.bodies.ptr(index);
}
body2D::ptr body2D::as_ptr()
{
    return world.bodies.ptr(index);
}

float body2D::kinetic_energy() const
{
    return 0.5f * (m_props.nondynamic.mass * glm::length2(velocity) +
                   angular_velocity * angular_velocity * m_props.nondynamic.inertia);
}

void body2D::add_force_at(const glm::vec2 &force, const glm::vec2 &at)
{
    impulse_force += force;
    impulse_torque += kit::cross2D(at, force);
}

void body2D::apply_simulation_force(const glm::vec2 &force)
{
    m_force += force;
}
void body2D::apply_simulation_force_at(const glm::vec2 &force, const glm::vec2 &at)
{
    m_force += force;
    m_torque += kit::cross2D(at, force);
}
void body2D::apply_simulation_torque(const float torque)
{
    m_torque += torque;
}

const glm::vec2 &body2D::force() const
{
    return m_force;
}
float body2D::torque() const
{
    return m_torque;
}
const glm::vec2 &body2D::charge_centroid() const
{
    return m_charge_centroid;
}

void body2D::translate(const glm::vec2 &dpos)
{
    m_transform.position += dpos;
    m_props.position += dpos;
    update_colliders();
}
void body2D::rotate(const float dangle)
{
    m_transform.rotation += dangle;
    update_colliders();
}

const kit::transform2D<float> &body2D::transform() const
{
    return m_transform;
}

const glm::vec2 &body2D::centroid() const
{
    return m_transform.position;
}
const glm::vec2 &body2D::position() const
{
    return m_props.position;
}
const glm::vec2 &body2D::origin() const
{
    return m_transform.origin;
}

glm::vec2 body2D::velocity_at(const glm::vec2 &at) const
{
    return velocity + angular_velocity * glm::vec2(-at.y, at.x); // v + cross(w, at)
}

float body2D::rotation() const
{
    return m_transform.rotation;
}

void body2D::centroid(const glm::vec2 &centroid)
{
    m_props.position += centroid - m_transform.position;
    m_transform.position = centroid;
    update_colliders();
}
void body2D::position(const glm::vec2 &position)
{
    m_transform.position += position - m_props.position;
    m_props.position = position;
    update_colliders();
}
void body2D::origin(const glm::vec2 &origin)
{
    m_transform.origin = origin;
    update_colliders();
}
void body2D::rotation(const float rotation)
{
    m_props.position =
        m_transform.position + kit::transform2D<float>::rotation_matrix(rotation - m_transform.rotation) *
                                   (m_props.position - m_transform.position);
    m_transform.rotation = rotation;
    update_colliders();
}

void body2D::match_position_with_centroid()
{
    m_props.position = m_transform.position;
}

void body2D::mass(const float mass)
{
    m_props.nondynamic.inertia *= mass / m_props.nondynamic.mass;
    m_props.nondynamic.mass = mass;
    if (is_dynamic())
    {
        m_props.dynamic.mass = mass;
        m_props.dynamic.inertia = m_props.nondynamic.inertia;
    }
}

} // namespace ppx