#include "entity2D.hpp"

namespace physics
{
    const geo::box2D &entity2D::bounding_box() const { return m_bbox; }
    const geo::polygon2D &entity2D::shape() const { return m_shape; }
}
