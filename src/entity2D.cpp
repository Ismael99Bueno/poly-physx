#include "entity2D.hpp"

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

    const geo::box2D &entity2D::bounding_box() const { return m_bbox; }
    const geo::polygon2D &entity2D::shape() const { return m_shape; }
}
