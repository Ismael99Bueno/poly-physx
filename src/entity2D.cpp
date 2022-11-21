#include "entity2D.hpp"
#include "force2D.hpp"
#include "interaction2D.hpp"
#include "debug.h"

namespace physics
{
    entity2D::entity2D(const body2D &body,
                       const std::vector<vec2> &vertices) : body2D(body),
                                                            m_shape(body.pos(), vertices) {}

    entity2D::entity2D(const vec2 &pos,
                       const vec2 &vel,
                       const float angpos, const float angvel,
                       const float mass, const float charge,
                       const std::vector<vec2> &vertices) : body2D(pos, vel, angpos,
                                                                   angvel, mass, charge),
                                                            m_shape(pos, vertices),
                                                            m_bbox() {}

    void entity2D::retrieve(const utils::const_vec_ptr &buffer)
    {
        DBG_EXIT_IF(!buffer, "Cannot retrieve from a null buffer.\n")
        pos({buffer[0], buffer[1]});
        vel({buffer[3], buffer[4]});
        angpos(buffer[2]);
        angvel(buffer[5]);
    }

    void entity2D::retrieve() { retrieve(m_buffer); }
    void entity2D::dispatch() const
    {
        m_buffer[0] = m_pos.x;
        m_buffer[1] = m_pos.y;
        m_buffer[2] = m_angpos;
        m_buffer[3] = m_vel.x;
        m_buffer[4] = m_vel.y;
        m_buffer[5] = m_angvel;
    }

    void entity2D::add_force(const vec2 &force) { m_force += force; }
    void entity2D::add_torque(const float torque) { m_torque += torque; }

    void entity2D::include(const force2D &force) { m_forces.insert(&force); }
    void entity2D::include(const interaction2D &inter) { m_inters.insert(&inter); }

    void entity2D::exclude(const force2D &force) { m_forces.erase(&force); }
    void entity2D::exclude(const interaction2D &inter) { m_inters.erase(&inter); }

    bool entity2D::contains(const force2D &force) const { return m_forces.find(&force) != m_forces.end(); }
    bool entity2D::contains(const interaction2D &inter) const { return m_inters.find(&inter) != m_inters.end(); }

    std::pair<vec2, float> entity2D::force() const
    {
        if (!m_dynamic)
            return {{0.f, 0.f}, 0.f};
        vec2 force;
        float torque = 0.f;
        for (const force2D *f2D : m_forces)
        {
            const auto [f, t] = f2D->force(*this);
            force += f;
            torque += t;
        }
        for (const interaction2D *i2D : m_inters)
            for (const const_entity_ptr &e : i2D->entities())
                if (&(*e) != this)
                {
                    const auto [f, t] = i2D->force(*this, *e);
                    force += f;
                    torque += t;
                }
        return {force, torque};
    }

    const geo::box2D &entity2D::bounding_box() const { return m_bbox; }
    const geo::polygon2D &entity2D::shape() const { return m_shape; }

    const geo::polygon2D &entity2D::shape(const geo::polygon2D &poly)
    {
        m_shape = poly;
        m_shape.pos(m_pos);
        m_shape.rotation(m_angpos);
        m_bbox.bound(m_shape.vertices(), m_shape.centroid());
        return m_shape;
    }

    const vec2 &entity2D::pos() const { return m_pos; }

    void entity2D::pos(const vec2 &pos)
    {
        m_pos = pos;
        m_shape.pos(pos);
        m_bbox.recentre(m_shape.centroid());
        // m_bbox.bound(m_shape.vertices(), m_shape.centroid());
    }

    float entity2D::angpos() const { return m_angpos; }

    void entity2D::angpos(const float angpos)
    {
        m_angpos = angpos;
        m_shape.rotation(angpos);
        m_bbox.bound(m_shape.vertices(), m_shape.centroid());
    }

    float entity2D::inertia() const { return m_shape.inertia() * m_mass; }

    bool entity2D::dynamic() const { return m_dynamic; }
    void entity2D::dynamic(bool dynamic) { m_dynamic = dynamic; }
}
