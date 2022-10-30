#ifndef ENTITY2D_HPP
#define ENTITY2D_HPP

#include "state.hpp"
#include "integrator.hpp"
#include "box2D.hpp"
#include "polygon2D.hpp"

namespace physics
{
    class entity2D
    {
    public:
        const geo::box2D &bounding_box() const;
        const geo::polygon2D &shape() const;

    private:
        rk::state m_state;
        rk::integrator m_integ;
        geo::box2D m_bbox;
        geo::polygon2D m_shape;
    };
}

#endif