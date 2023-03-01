#ifndef ENTITY2D_HPP
#define ENTITY2D_HPP

#include "aabb2D.hpp"
#include "polygon2D.hpp"
#include "saveable.hpp"
#include "state.hpp"

namespace phys
{
    class entity2D : public ini::saveable
    {
    public:
        entity2D(const alg::vec2 &pos,
                 const alg::vec2 &vel,
                 float angpos, float angvel,
                 float mass, float charge,
                 const std::vector<alg::vec2> &vertices,
                 bool kynematic);

        void retrieve();
        void dispatch() const;

        void add_force(const alg::vec2 &force);
        void add_torque(float torque);

        const alg::vec2 &force() const;
        float torque() const;
        const alg::vec2 &added_force() const;
        float added_torque() const;

        const geo::aabb2D &aabb() const;

        const geo::polygon2D &shape() const;
        void shape(const std::vector<alg::vec2> &vertices);

        std::size_t index() const;
        std::size_t id() const;

        float inertia() const;

        bool kynematic() const;
        void kynematic(bool kynematic);

        void translate(const alg::vec2 &dpos);
        void rotate(float dangle);

        void write(ini::output &out) const override;
        void read(ini::input &in) override;

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
        geo::aabb2D m_aabb;
        geo::polygon2D m_shape;
        rk::state *m_state = nullptr;
        alg::vec2 m_vel, m_force, m_added_force;
        std::size_t m_index = 0, m_id;
        float m_angvel, m_torque, m_added_torque = 0.f, m_mass, m_charge;
        bool m_kynematic;

        static std::size_t s_id;

        void retrieve(const std::vector<float> &vars_buffer);
        friend class engine2D;
    };

    bool operator==(const entity2D &lhs, const entity2D &rhs);
    bool operator!=(const entity2D &lhs, const entity2D &rhs);
}

#endif