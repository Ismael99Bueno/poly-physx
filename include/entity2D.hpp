#ifndef ENTITY2D_HPP
#define ENTITY2D_HPP

#include "body2D.hpp"
#include "box2D.hpp"
#include "polygon2D.hpp"
#include "vec_ptr.hpp"

namespace phys
{
    class entity2D : public body2D
    {
    public:
        entity2D(const body2D &body,
                 const std::vector<alg::vec2> &vertices = geo::polygon2D::box(1.f));

        entity2D(const alg::vec2 &pos = {0.f, 0.f},
                 const alg::vec2 &vel = {0.f, 0.f},
                 float angpos = 0.f, float angvel = 0.f,
                 float mass = 1.f, float charge = 1.f,
                 const std::vector<alg::vec2> &vertices = geo::polygon2D::box(1.f));

        void retrieve();
        void dispatch() const;

        void add_force(const alg::vec2 &force);
        void add_torque(float torque);

        alg::vec2 added_force() const;
        float added_torque() const;

        const geo::box2D &bounding_box() const;
        const geo::polygon2D &shape() const;

        const geo::polygon2D &shape(const geo::polygon2D &poly);

        const alg::vec2 &pos() const override;
        void pos(const alg::vec2 &pos) override;
        float angpos() const override;
        void angpos(float angpos) override;

        float inertia() const;

        bool dynamic() const;
        void dynamic(bool dynamic);

    private:
        geo::box2D m_bbox;
        geo::polygon2D m_shape;
        utils::vec_ptr m_buffer;
        alg::vec2 m_added_force;
        float m_added_torque = 0.f;
        bool m_dynamic = true;

        void retrieve(const utils::const_vec_ptr &buffer);
        friend class engine2D;
    };
}

#endif