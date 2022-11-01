#include "entity2D.hpp"
#include "debug.h"

namespace physics
{
    void entity2D::retrieve(const utils::vec_ptr &buffer)
    {
        DBG_EXIT_IF(!buffer, "Cannot retrieve from a null buffer.\n")
        m_pos = {buffer[0], buffer[1]};
        m_vel = {buffer[3], buffer[4]};
        m_angpos = buffer[2];
        m_angvel = buffer[5];
    }

    void entity2D::retrieve() { retrieve(m_buffer); }

    void entity2D::add(const force2D &force) { m_forces.insert(&force); }
    void entity2D::add(const interaction2D &inter) { m_inters.insert(&inter); }

    void entity2D::remove(const force2D &force) { m_forces.erase(&force); }
    void entity2D::remove(const interaction2D &inter) { m_inters.erase(&inter); }

    bool entity2D::contains(const force2D &force) const { return m_forces.find(&force) != m_forces.end(); }
    bool entity2D::contains(const interaction2D &inter) const { return m_inters.find(&inter) != m_inters.end(); }

    const geo::box2D &entity2D::bounding_box() const { return m_bbox; }
    const geo::polygon2D &entity2D::shape() const { return m_shape; }
}
