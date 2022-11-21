#ifndef ENTITY2D_HPP
#define ENTITY2D_HPP

#include "body2D.hpp"
#include "box2D.hpp"
#include "polygon2D.hpp"
#include "vec_ptr.hpp"
#include <unordered_set>
#include <utility>

namespace phys
{
    class force2D;
    class interaction2D;
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

        bool contains(const force2D &force) const;
        bool contains(const interaction2D &inter) const;

        void add_force(const alg::vec2 &force);
        void add_torque(float torque);
        std::pair<alg::vec2, float> force() const;

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
        alg::vec2 m_force;
        float m_torque = 0.f;
        bool m_dynamic = true;

        std::unordered_set<const force2D *> m_forces;
        std::unordered_set<const interaction2D *> m_inters;

        void retrieve(const utils::const_vec_ptr &buffer);

        void include(const force2D &force);
        void include(const interaction2D &inter);

        void exclude(const force2D &force);
        void exclude(const interaction2D &inter);

        friend class force2D;
        friend class interaction2D;
        friend class engine2D;
    };
}

#endif