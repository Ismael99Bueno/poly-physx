#ifndef ENTITY2D_HPP
#define ENTITY2D_HPP

#include "body2D.hpp"
#include "integrator.hpp"
#include "box2D.hpp"
#include "polygon2D.hpp"
#include "vec_ptr.hpp"

namespace physics
{
    class entity2D : public body2D
    {
    public:
        using body2D::body2D;

        void retrieve(const utils::vec_ptr &buffer);
        void retrieve();

        const geo::box2D &bounding_box() const;
        const geo::polygon2D &shape() const;

    private:
        geo::box2D m_bbox;
        geo::polygon2D m_shape;
        utils::vec_ptr m_buffer;
    };
}

#endif