#ifndef ENTITY2D_HPP
#define ENTITY2D_HPP

#include "box2D.hpp"
#include "polygon2D.hpp"
#include "vec_ptr.hpp"

namespace phys
{
    class entity2D
    {
    public:
        entity2D(const alg::vec2 &pos = {0.f, 0.f},
                 const alg::vec2 &vel = {0.f, 0.f},
                 float angpos = 0.f, float angvel = 0.f,
                 float mass = 1.f, float charge = 1.f,
                 const std::vector<alg::vec2> &vertices = geo::polygon2D::box(1.f));

        void retrieve();
        void dispatch() const;

        void add_force(const alg::vec2 &force);
        void add_torque(float torque);

        const alg::vec2 &force() const;
        float torque() const;
        const alg::vec2 &added_force() const;
        float added_torque() const;

        const geo::box2D &bounding_box() const;

        const geo::polygon2D &shape() const;
        void shape(const geo::polygon2D &poly);

        std::size_t index() const;
        std::uint64_t id() const;

        float inertia() const;

        bool dynamic() const;
        void dynamic(bool dynamic);

        void translate(const alg::vec2 &dpos);
        void rotate(float dangle);

        const alg::vec2 &pos() const;
        const alg::vec2 &vel() const;
        const alg::vec2 vel_at(const alg::vec2 &at) const;
        float angpos() const;
        float angvel() const;
        float mass() const;
        float charge() const;

        void pos(const alg::vec2 &pos);
        void vel(const alg::vec2 &vel);
        void angpos(float angpos);
        void angvel(float angvel);
        void mass(float mass);
        void charge(float charge);

    private:
        geo::box2D m_bbox;
        geo::polygon2D m_shape;
        utils::vec_ptr m_buffer;
        alg::vec2 m_vel, m_force, m_added_force;
        std::size_t m_index = 0;
        std::uint64_t m_id;
        float m_angvel, m_torque, m_added_torque = 0.f, m_mass, m_charge;
        bool m_dynamic = true;

        static std::size_t s_id;

        void retrieve(const utils::const_vec_ptr &buffer);
        friend class engine2D;
    };
}

#endif