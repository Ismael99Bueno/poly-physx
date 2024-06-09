#include "ppx/internal/pch.hpp"
#include "ppx/body/body2D.hpp"
#include "ppx/collider/collider2D.hpp"
#include "ppx/world2D.hpp"
#include "ppx/behaviours/force2D.hpp"
#include "ppx/behaviours/interaction2D.hpp"

namespace ppx
{
body2D::body2D(world2D &world, const body2D::specs &spc)
    : kit::indexable(world.bodies.size()), worldref2D(world),
      m_state({transform2D{kit::transform2D<float>::builder().position(spc.position).rotation(spc.rotation).build()},
               spc.position, glm::vec2(0.f), spc.velocity, spc.angular_velocity}),
      m_charge_centroid(spc.position), m_charge(spc.props.charge), m_type(spc.props.type)
{
    mass(spc.props.mass);
}

void body2D::metadata::remove_joint(const joint2D *joint)
{
    for (std::size_t i = 0; i < joints.size(); i++)
        if (joints[i] == joint)
        {
            joints.erase(joints.begin() + i);
            return;
        }
    KIT_WARN("Joint not found in body metadata")
}

void body2D::metadata::remove_contact(const contact2D *contact)
{
    for (std::size_t i = 0; i < contacts.size(); i++)
        if (contacts[i] == contact)
        {
            contacts.erase(contacts.begin() + i);
            return;
        }
    KIT_WARN("Contact not found in body metadata")
}

const collider2D *body2D::operator[](const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_colliders.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_colliders.size());
    return m_colliders[index];
}
collider2D *body2D::operator[](const std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_colliders.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_colliders.size());
    return m_colliders[index];
}

collider2D *body2D::add(const ppx::specs::collider2D &spc)
{
    return world.colliders.add(this, spc);
}

bool body2D::remove(const std::size_t index)
{
    if (index >= m_colliders.size())
        return false;
    return world.colliders.remove(m_colliders[index]);
}

bool body2D::remove(collider2D *collider)
{
    for (collider2D *c : *this)
        if (c == collider)
            return world.colliders.remove(collider);
    return false;
}
bool body2D::contains(const collider2D *collider) const
{
    for (const collider2D *c : *this)
        if (c == collider)
            return true;
    return false;
}

void body2D::clear()
{
    for (std::size_t i = m_colliders.size() - 1; i < m_colliders.size() && i >= 0; i--)
        remove(i);
}

bool body2D::empty() const
{
    return m_colliders.empty();
}
std::size_t body2D::size() const
{
    return m_colliders.size();
}

void body2D::awake(const bool even_if_non_dynamic)
{
    if (!m_awake_allowed)
        return;
    if (meta.island)
        meta.island->awake();
    else if (even_if_non_dynamic && world.islands.enabled())
    {
        for (joint2D *joint : meta.joints)
            joint->other(this)->awake();
        for (contact2D *contact : meta.contacts)
            contact->other(this)->awake();
    }
}
bool body2D::asleep() const
{
    if (!world.islands.enabled())
        return false;
    if (meta.island)
        return meta.island->asleep();
    if (is_kinematic())
        return kinetic_energy() < world.islands.params.lower_sleep_energy_threshold;
    return true;
}
void body2D::stop_all_motion()
{
    m_state.velocity = glm::vec2(0.f);
    m_state.angular_velocity = 0.f;
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
    m_awake_allowed = false;
    begin_spatial_update();
    centroid({vars_buffer[idx + 0], vars_buffer[idx + 1]});
    rotation(vars_buffer[idx + 2]);
    end_spatial_update();

    m_state.velocity = {vars_buffer[idx + 3], vars_buffer[idx + 4]};
    m_state.angular_velocity = vars_buffer[idx + 5];
    m_awake_allowed = true;
}

void body2D::full_update()
{
    update_centroids();
    update_inertia();
    update_colliders();
    awake(true);
}

void body2D::update_colliders()
{
    if (m_spatial_update)
        return;
    for (collider2D *collider : *this)
    {
        shape2D &shape = collider->mutable_shape();
        KIT_ASSERT_ERROR(!shape.updating(), "Cannot update collider while it is already being updated")
        shape.update();
    }
}
void body2D::update_centroids()
{
    if (m_density_update)
        return;
    if (empty())
    {
        m_charge_centroid = m_state.gposition;
        m_state.centroid.position(m_state.gposition);
        return;
    }
    glm::vec2 centroid{0.f};
    float artificial_mass = 0.f;

    glm::vec2 charge_centroid{0.f};
    float artificial_charge = 0.f;

    for (const collider2D *collider : *this)
    {
        const shape2D &shape = collider->shape();

        const float cmass = collider->density() * shape.area();
        centroid += cmass * shape.gcentroid();
        artificial_mass += cmass;

        const float ccharge = glm::abs(collider->charge_density()) * shape.area();
        charge_centroid += ccharge * shape.gcentroid();
        artificial_charge += ccharge;
    }
    m_charge_centroid = charge_centroid / artificial_charge;
    centroid /= artificial_mass;

    const glm::vec2 diff = m_state.centroid.position() - centroid;
    m_state.centroid.position(centroid);
    for (collider2D *collider : *this)
        collider->mutable_shape().gtranslate(diff);
    m_state.lposition = local_centroid_point(m_state.gposition);
}
void body2D::update_inertia()
{
    if (m_density_update)
        return;

    m_props.nondynamic.inertia = 0.f;
    if (empty())
    {
        m_props.nondynamic.inv_inertia = 0.f;
        m_props.dynamic.inertia = is_dynamic() ? 0.f : FLT_MAX;
        m_props.dynamic.inv_inertia = 0.f;
        return;
    }

    float artificial_mass = 0.f;
    for (const collider2D *collider : *this)
    {
        const shape2D &shape = collider->shape();
        const float cmass = collider->density() * shape.area();
        const float dist2 = glm::length2(shape.gcentroid() - m_state.centroid.position());

        m_props.nondynamic.inertia += cmass * (dist2 + shape.inertia());
        artificial_mass += cmass;
    }
    m_props.nondynamic.inertia *= m_props.nondynamic.mass / artificial_mass;
    m_props.nondynamic.inv_inertia = 1.f / m_props.nondynamic.inertia;
    if (is_dynamic())
    {
        m_props.dynamic.inertia = m_props.nondynamic.inertia;
        m_props.dynamic.inv_inertia = m_props.nondynamic.inv_inertia;
    }
}

bool body2D::density_updating() const
{
    return m_density_update;
}
bool body2D::spatial_updating() const
{
    return m_spatial_update;
}

void body2D::prepare_constraint_states()
{
    meta.ctr_state = m_state;
    if (is_dynamic() && world.semi_implicit_integration)
    {
        meta.ctr_state.velocity += m_props.dynamic.inv_mass * m_force * world.rk_substep_timestep();
        meta.ctr_state.angular_velocity += m_props.dynamic.inv_inertia * m_torque * world.rk_substep_timestep();
    }
}

const std::vector<joint2D *> &body2D::joints() const
{
    return meta.joints;
}
const std::vector<contact2D *> &body2D::contacts() const
{
    return meta.contacts;
}

bool body2D::checksum() const
{
    const std::unordered_set<const collider2D *> colliders{m_colliders.begin(), m_colliders.end()};
    const std::unordered_set<const joint2D *> joints{meta.joints.begin(), meta.joints.end()};
    const std::unordered_set<const contact2D *> contacts{meta.contacts.begin(), meta.contacts.end()};

    KIT_ASSERT_ERROR(colliders.size() == m_colliders.size(), "Duplicate colliders in body")
    KIT_ASSERT_ERROR(joints.size() == meta.joints.size(), "Duplicate joints in body")
    KIT_ASSERT_ERROR(contacts.size() == meta.contacts.size(), "Duplicate contacts in body")
    return colliders.size() == m_colliders.size() && joints.size() == meta.joints.size() &&
           contacts.size() == meta.contacts.size();
}

void body2D::reset_simulation_forces()
{
    m_force = glm::vec2(0.f);
    m_torque = 0.f;
}

bool body2D::is_dynamic() const
{
    return m_type == btype::DYNAMIC;
}
bool body2D::is_kinematic() const
{
    return m_type == btype::KINEMATIC;
}
bool body2D::is_static() const
{
    return m_type == btype::STATIC;
}

bool body2D::joint_prevents_collision(const body2D *body) const
{
    for (const joint2D *joint : body->meta.joints)
        if (!joint->bodies_collide && joint->contains(this))
            return true;
    return false;
}
bool body2D::attached_to(const body2D *body) const
{
    for (const joint2D *joint : body->meta.joints)
        if (joint->contains(this))
            return true;
    return false;
}
bool body2D::attached_to(const joint2D *joint) const
{
    return joint->contains(this);
}

body2D::btype body2D::type() const
{
    return m_type;
}
void body2D::type(btype type)
{
    if (type == m_type)
        return;

    const bool non_dynamic_change =
        (type == btype::KINEMATIC && is_static()) || (type == btype::STATIC && is_kinematic());

    if (non_dynamic_change)
    {
        m_type = type;
        reset_dynamic_properties();
        return;
    }

    if (type == btype::DYNAMIC)
    {
        m_type = type;
        reset_dynamic_properties();
        if (world.islands.enabled())
        {
            island2D *island = world.islands.create();
            island->add_body(this);
        }
        return;
    }
    for (std::size_t i = meta.joints.size() - 1; i < meta.joints.size() && i >= 0; i--)
    {
        joint2D *joint = meta.joints[i];
        if (!joint->other(this)->is_dynamic())
            world.joints.remove(joint);
    }

    for (collider2D *collider : *this)
        world.collisions.contacts()->remove_any_contacts_with(collider);
    if (meta.island)
        meta.island->remove_body(this);

    m_type = type;
    reset_dynamic_properties();
}

float body2D::kinetic_energy() const
{
    return 0.5f * (m_props.nondynamic.mass * glm::length2(m_state.velocity) +
                   m_state.angular_velocity * m_state.angular_velocity * m_props.nondynamic.inertia);
}

glm::vec2 body2D::local_centroid_point(const glm::vec2 &gpoint) const
{
    return m_state.local_centroid_point(gpoint);
}
glm::vec2 body2D::global_centroid_point(const glm::vec2 &lpoint) const
{
    return m_state.global_centroid_point(lpoint);
}

glm::vec2 body2D::local_position_point(const glm::vec2 &gpoint) const
{
    return m_state.local_position_point(gpoint);
}
glm::vec2 body2D::global_position_point(const glm::vec2 &lpoint) const
{
    return m_state.global_position_point(lpoint);
}

glm::vec2 body2D::local_vector(const glm::vec2 &gvector) const
{
    return m_state.local_vector(gvector);
}
glm::vec2 body2D::global_vector(const glm::vec2 &lvector) const
{
    return m_state.global_vector(lvector);
}

void body2D::ladd_force_at(const glm::vec2 &force, const glm::vec2 &lpoint)
{
    gadd_force_at(force, global_centroid_point(lpoint));
}
void body2D::gadd_force_at(const glm::vec2 &force, const glm::vec2 &gpoint)
{
    m_instant_force += force;
    m_instant_torque += kit::cross2D(gpoint - m_state.centroid.position(), force);
    awake();
}
void body2D::add_force(const glm::vec2 &force)
{
    m_instant_force += force;
    awake();
}

const glm::vec2 &body2D::instant_force() const
{
    return m_instant_force;
}
const glm::vec2 &body2D::persistent_force() const
{
    return m_persistent_force;
}
float body2D::instant_torque() const
{
    return m_instant_torque;
}
float body2D::persistent_torque() const
{
    return m_persistent_torque;
}

void body2D::persistent_force(const glm::vec2 &force)
{
    m_persistent_force = force;
    awake();
}
void body2D::instant_force(const glm::vec2 &force)
{
    m_instant_force = force;
    awake();
}

void body2D::persistent_torque(const float torque)
{
    m_persistent_torque = torque;
    awake();
}
void body2D::instant_torque(const float torque)
{
    m_instant_torque = torque;
    awake();
}

void body2D::apply_simulation_force(const glm::vec2 &force)
{
    m_force += force;
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

const glm::vec2 &body2D::velocity() const
{
    return m_state.velocity;
}

float body2D::angular_velocity() const
{
    return m_state.angular_velocity;
}

void body2D::velocity(const glm::vec2 &velocity)
{
    m_state.velocity = velocity;
    awake(true);
}
void body2D::angular_velocity(const float angular_velocity)
{
    m_state.angular_velocity = angular_velocity;
    awake(true);
}

void body2D::translate(const glm::vec2 &dpos)
{
    m_state.centroid.translate(dpos);
    m_state.gposition += dpos;
    m_charge_centroid += dpos;
    update_colliders();
    awake(true);
}
void body2D::rotate(const float dangle)
{
    rotation(m_state.centroid.rotation() + dangle);
}

const transform2D &body2D::centroid_transform() const
{
    return m_state.centroid;
}
void body2D::centroid_transform(const transform2D &centroid)
{
    const glm::vec2 dpos = centroid.position() - m_state.centroid.position();
    m_state.gposition += dpos;
    m_charge_centroid += dpos;
    m_state.centroid = centroid;
    update_colliders();
    awake(true);
}

const glm::vec2 &body2D::centroid() const
{
    return m_state.centroid.position();
}

const glm::vec2 &body2D::lposition() const
{
    return m_state.lposition;
}
const glm::vec2 &body2D::gposition() const
{
    return m_state.gposition;
}

const glm::vec2 &body2D::origin() const
{
    return m_state.centroid.origin();
}

glm::vec2 body2D::lvelocity_at_from_centroid(const glm::vec2 &lpoint) const
{
    return m_state.lvelocity_at_from_centroid(lpoint);
}
glm::vec2 body2D::lvelocity_at_from_position(const glm::vec2 &lpoint) const
{
    return m_state.lvelocity_at_from_position(lpoint);
}
glm::vec2 body2D::gvelocity_at(const glm::vec2 &gpoint) const
{
    return m_state.gvelocity_at(gpoint);
}
glm::vec2 body2D::velocity_at_centroid_offset(const glm::vec2 &offset) const
{
    return m_state.velocity_at_centroid_offset(offset);
}
glm::vec2 body2D::velocity_at_position_offset(const glm::vec2 &offset) const
{
    return m_state.velocity_at_position_offset(offset);
}

float body2D::charge() const
{
    return m_charge;
}
void body2D::charge(const float charge)
{
    m_charge = charge;
    awake();
}

float body2D::rotation() const
{
    return m_state.centroid.rotation();
}

void body2D::centroid(const glm::vec2 &centroid)
{
    const glm::vec2 dpos = centroid - m_state.centroid.position();
    m_state.gposition += dpos;
    m_charge_centroid += dpos;
    m_state.centroid.position(centroid);
    update_colliders();
    awake(true);
}

void body2D::gposition(const glm::vec2 &gposition)
{
    const glm::vec2 dpos = gposition - m_state.gposition;
    m_state.centroid.translate(dpos);
    m_charge_centroid += dpos;
    m_state.gposition = gposition;
    update_colliders();
    awake(true);
}
void body2D::origin(const glm::vec2 &origin)
{
    m_state.centroid.origin(origin);
    update_colliders();
    awake(true);
}
void body2D::rotation(const float rotation)
{
    const glm::mat2 rmat = kit::transform2D<float>::rotation_matrix(rotation - m_state.centroid.rotation());
    m_state.gposition = m_state.centroid.position() + rmat * (m_state.gposition - m_state.centroid.position());
    m_charge_centroid = m_state.centroid.position() + rmat * (m_charge_centroid - m_state.centroid.position());
    m_state.centroid.rotation(rotation);
    update_colliders();
    awake(true);
}

void body2D::mass(const float mass)
{
    const float pmass = m_props.nondynamic.mass;

    m_props.nondynamic.mass = mass;
    m_props.nondynamic.inv_mass = 1.f / mass;
    if (kit::approaches_zero(m_props.nondynamic.inertia))
        update_inertia();
    else if (!kit::approaches_zero(pmass))
    {
        const float ratio = mass / m_props.nondynamic.mass;
        m_props.nondynamic.inertia *= ratio;
        m_props.nondynamic.inv_inertia /= ratio;
    }
    reset_dynamic_properties();
    awake();
}

void body2D::reset_dynamic_properties()
{
    if (is_dynamic())
        m_props.dynamic = m_props.nondynamic;
    else
    {
        m_props.dynamic.mass = FLT_MAX;
        m_props.dynamic.inv_mass = 0.f;
        m_props.dynamic.inertia = FLT_MAX;
        m_props.dynamic.inv_inertia = 0.f;
        if (is_static() || (is_dynamic() && asleep()))
            stop_all_motion();
    }
}

} // namespace ppx