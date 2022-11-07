#include "entity2D.hpp"
#include "force2D.hpp"
#include "interaction2D.hpp"
#include "debug.h"

namespace physics
{
    entity2D::entity2D(const vec2 &pos,
                       const vec2 &vel,
                       const float angpos, const float angvel,
                       const float mass, const float charge) : body2D(pos, vel, angpos,
                                                                      angvel, mass, charge),
                                                               m_bbox(m_shape) {}

    void entity2D::retrieve(const utils::const_vec_ptr &buffer)
    {
        DBG_EXIT_IF(!buffer, "Cannot retrieve from a null buffer.\n")
        m_pos = {buffer[0], buffer[1]};
        m_vel = {buffer[3], buffer[4]};
        m_angpos = buffer[2];
        m_angvel = buffer[5];
        m_shape.move(m_pos);
        m_shape.rotation(m_angpos);
    }

    void entity2D::retrieve() { retrieve(m_buffer); }

    void entity2D::add_force(const vec2 &force) { m_accel.first += force / m_mass; }
    void entity2D::add_torque(const float torque) { m_accel.second += torque / m_mass; }

    void entity2D::add(const force2D &force) { m_forces.insert(&force); }
    void entity2D::add(const interaction2D &inter) { m_inters.insert(&inter); }

    void entity2D::remove(const force2D &force) { m_forces.erase(&force); }
    void entity2D::remove(const interaction2D &inter) { m_inters.erase(&inter); }

    bool entity2D::contains(const force2D &force) const { return m_forces.find(&force) != m_forces.end(); }
    bool entity2D::contains(const interaction2D &inter) const { return m_inters.find(&inter) != m_inters.end(); }

    std::pair<vec2, float> entity2D::accel() const
    {
        vec2 linaccel;
        float angaccel = 0.f;
        for (const force2D *force : m_forces)
        {
            const auto [lin, ang] = force->acceleration(*this);
            linaccel += lin;
            angaccel += ang;
        }
        for (const interaction2D *inter : m_inters)
            for (const const_entity_ptr &e : inter->entities())
                if (&(*e) != this)
                {
                    const auto [lin, ang] = inter->acceleration(*this, *e);
                    linaccel += lin;
                    angaccel += ang;
                }
        return {linaccel, angaccel};
    }

    const geo::box2D &entity2D::bounding_box() const { return m_bbox; }
    const geo::polygon2D &entity2D::shape() const { return m_shape; }

    geo::box2D &entity2D::bounding_box() { return m_bbox; }
    geo::polygon2D &entity2D::shape() { return m_shape; }

    geo::polygon2D &entity2D::shape(const geo::polygon2D &poly)
    {
        m_shape = poly;
        m_bbox.bound();
        return m_shape;
    }
}
