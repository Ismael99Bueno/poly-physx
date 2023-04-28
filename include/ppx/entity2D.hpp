#ifndef ENTITY2D_HPP
#define ENTITY2D_HPP

#include "geo/aabb2D.hpp"
#include "geo/polygon.hpp"
#include "geo/circle.hpp"
#include "ini/saveable.hpp"
#include "rk/state.hpp"
#include "ppx/entity_events.hpp"
#include <variant>

namespace ppx
{
    class entity2D : public ini::saveable
    {
    public:
        enum shape_type
        {
            POLYGON = 0,
            CIRCLE = 1
        };

        entity2D(const std::variant<geo::polygon, geo::circle> &shape,
                 const glm::vec2 &pos = glm::vec2(0.f),
                 const glm::vec2 &vel = glm::vec2(0.f),
                 float angpos = 0.f, float angvel = 0.f,
                 float mass = 1.f, float charge = 1.f,
                 bool kinematic = true);
        entity2D(const std::vector<glm::vec2> &vertices,
                 const glm::vec2 &pos = glm::vec2(0.f),
                 const glm::vec2 &vel = glm::vec2(0.f),
                 float angpos = 0.f, float angvel = 0.f,
                 float mass = 1.f, float charge = 1.f,
                 bool kinematic = true);
        entity2D(float radius,
                 const glm::vec2 &pos = glm::vec2(0.f),
                 const glm::vec2 &vel = glm::vec2(0.f),
                 float angpos = 0.f, float angvel = 0.f,
                 float mass = 1.f, float charge = 1.f,
                 bool kinematic = true);

        entity2D(const glm::vec2 &pos = glm::vec2(0.f),
                 const glm::vec2 &vel = glm::vec2(0.f),
                 float angpos = 0.f, float angvel = 0.f,
                 float mass = 1.f, float charge = 1.f,
                 bool kinematic = true);

        void retrieve();
        void dispatch() const;
        float kinetic_energy() const;

        void add_force(const glm::vec2 &force);
        void add_torque(float torque);

        const glm::vec2 &force() const;
        float torque() const;
        const glm::vec2 &added_force() const;
        float added_torque() const;

        const geo::shape2D &shape() const;

        template <typename T>
        const T &shape() const;

        template <typename T>
        const T *shape_if() const;

        void shape(const std::vector<glm::vec2> &vertices);
        void shape(float radius);
        void shape(const geo::polygon &poly);
        void shape(const geo::circle &c);

        shape_type type() const;

        std::size_t index() const;
        std::size_t id() const;

        float inertia() const;

        bool kinematic() const;
        void kinematic(bool kinematic);

        void translate(const glm::vec2 &dpos);
        void rotate(float dangle);

        void write(ini::output &out) const override;
        void read(ini::input &in) override;

        const entity_events &events() const;
        entity_events &events();

        const glm::vec2 &pos() const;
        const glm::vec2 &vel() const;
        const glm::vec2 vel_at(const glm::vec2 &at) const;
        float angpos() const;
        float angvel() const;
        float mass() const;
        float charge() const;

        void pos(const glm::vec2 &pos);
        void vel(const glm::vec2 &vel);
        void angpos(float angpos);
        void angvel(float angvel);
        void mass(float mass);
        void charge(float charge);

    private:
        struct non_copyable // So that the user can assign to references to entities in the vector without messing up sensitive data
        {
            non_copyable() = default;
            std::size_t index = 0, id = 0;
            non_copyable(const non_copyable &) {}
            non_copyable &operator=(const non_copyable &) { return *this; }
        };
        std::variant<geo::polygon, geo::circle> m_shape;
        rk::state *m_state = nullptr;
        glm::vec2 m_vel{0.f}, m_force{0.f}, m_added_force{0.f};
        non_copyable m_non_copyable;
        entity_events m_events;
        float m_angvel, m_torque, m_added_torque = 0.f, m_mass, m_charge;
        bool m_kinematic;

        geo::shape2D &get_shape();
        void retrieve(const std::vector<float> &vars_buffer);
        friend class engine2D;
    };

    bool operator==(const entity2D &lhs, const entity2D &rhs);
    bool operator!=(const entity2D &lhs, const entity2D &rhs);
}

#endif