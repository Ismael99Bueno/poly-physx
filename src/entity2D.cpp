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
        pos({buffer[0], buffer[1]});
        vel({buffer[3], buffer[4]});
        angpos(buffer[2]);
        angvel(buffer[5]);
    }

    void entity2D::retrieve() { retrieve(m_buffer); }

    void entity2D::add_force(const vec2 &force) { m_accel.first += force / m_mass; }
    void entity2D::add_torque(const float torque) { m_accel.second += torque / m_mass; }

    void entity2D::include(const force2D &force) { m_forces.insert(&force); }
    void entity2D::include(const interaction2D &inter) { m_inters.insert(&inter); }

    void entity2D::exclude(const force2D &force) { m_forces.erase(&force); }
    void entity2D::exclude(const interaction2D &inter) { m_inters.erase(&inter); }

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

    const geo::polygon2D &entity2D::shape(const geo::polygon2D &poly)
    {
        m_shape = poly;
        m_bbox.bound();
        return m_shape;
    }

    const vec2 &entity2D::pos() const { return m_pos; }

    void entity2D::pos(const vec2 &pos)
    {
        m_pos = pos;
        m_shape.pos(pos);
        m_bbox.recentre();
    }

    float entity2D::angpos() const { return m_angpos; }

    void entity2D::angpos(const float angpos)
    {
        m_angpos = angpos;
        m_shape.rotation(angpos);
        m_bbox.bound();
    }
}
