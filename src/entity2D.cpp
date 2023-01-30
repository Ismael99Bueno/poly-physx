#include "entity2D.hpp"
#include "force2D.hpp"
#include "interaction2D.hpp"
#include "debug.hpp"

namespace phys
{
    std::size_t entity2D::s_id = 0;
    entity2D::entity2D(const alg::vec2 &pos,
                       const alg::vec2 &vel,
                       const float angpos, const float angvel,
                       const float mass, const float charge,
                       const std::vector<alg::vec2> &vertices,
                       const bool dynamic) : m_aabb(),
                                             m_shape(pos, vertices),
                                             m_vel(vel),
                                             m_id(s_id++),
                                             m_angvel(angvel),
                                             m_mass(mass),
                                             m_charge(charge),
                                             m_dynamic(dynamic)
    {
        m_shape.rotate(angpos);
    }

    void entity2D::retrieve(const utils::const_vec_ptr &buffer)
    {
        DBG_ASSERT(buffer, "Cannot retrieve from a null buffer.\n")
        pos({buffer[0], buffer[1]});
        vel({buffer[3], buffer[4]});
        angpos(buffer[2]);
        angvel(buffer[5]);
    }

    void entity2D::retrieve() { retrieve(m_buffer); }
    void entity2D::dispatch() const
    {
        const alg::vec2 &pos = m_shape.centroid();
        const float angpos = m_shape.rotation();
        m_buffer[0] = pos.x;
        m_buffer[1] = pos.y;
        m_buffer[2] = angpos;
        m_buffer[3] = m_vel.x;
        m_buffer[4] = m_vel.y;
        m_buffer[5] = m_angvel;
    }

    void entity2D::add_force(const alg::vec2 &force) { m_added_force += force; }
    void entity2D::add_torque(const float torque) { m_added_torque += torque; }

    const alg::vec2 &entity2D::force() const { return m_force; }
    float entity2D::torque() const { return m_torque; }
    const alg::vec2 &entity2D::added_force() const { return m_added_force; }
    float entity2D::added_torque() const { return m_added_torque; }

    const geo::aabb2D &entity2D::aabb() const { return m_aabb; }
    const geo::polygon2D &entity2D::shape() const { return m_shape; }

    void entity2D::shape(const geo::polygon2D &poly)
    {
        m_shape = poly;
        m_aabb.bound(m_shape.vertices());
    }

    std::size_t entity2D::index() const { return m_index; }
    std::size_t entity2D::id() const { return m_id; }

    float entity2D::inertia() const { return m_shape.inertia() * m_mass; }

    bool entity2D::dynamic() const { return m_dynamic; }
    void entity2D::dynamic(const bool dynamic) { m_dynamic = dynamic; }

    void entity2D::translate(const alg::vec2 &dpos) { m_shape.translate(dpos); }
    void entity2D::rotate(const float dangle) { m_shape.rotate(dangle); }

    void entity2D::write(ini::output &out) const
    {
        m_shape.write(out);
        out.begin_section("velocity");
        m_vel.write(out);
        out.end_section();
        out.begin_section("added_force");
        m_added_force.write(out);
        out.end_section();
        out.write("mass", m_mass);
        out.write("charge", m_charge);
        out.write("dynamic", m_dynamic);
        out.write("angvel", m_angvel);
        out.write("added_torque", m_added_torque);
    }
    void entity2D::read(ini::input &in)
    {
        m_shape.read(in);
        m_aabb.bound(m_shape.vertices());

        in.begin_section("velocity");
        m_vel.read(in);
        in.end_section();
        in.begin_section("added_force");
        m_added_force.read(in);
        in.end_section();
        m_mass = in.readf("mass");
        m_charge = in.readf("charge");
        m_dynamic = (bool)in.readi("dynamic");
        m_angvel = in.readf("angvel");
        m_added_torque = in.readf("added_torque");
    }

    const alg::vec2 &entity2D::pos() const { return m_shape.centroid(); }
    const alg::vec2 &entity2D::vel() const { return m_vel; }
    const alg::vec2 entity2D::vel_at(const alg::vec2 &at) const { return m_vel + m_angvel * alg::vec2(-at.y, at.x); }

    float entity2D::angpos() const { return m_shape.rotation(); }
    float entity2D::angvel() const { return m_angvel; }

    float entity2D::mass() const { return m_mass; }
    float entity2D::charge() const { return m_charge; }

    void entity2D::pos(const alg::vec2 &pos)
    {
        m_shape.pos(pos);
        m_aabb.bound(m_shape.vertices());
    }
    void entity2D::vel(const alg::vec2 &vel) { m_vel = vel; }

    void entity2D::angpos(const float angpos)
    {
        m_shape.rotation(angpos);
        m_aabb.bound(m_shape.vertices());
    }
    void entity2D::angvel(const float angvel) { m_angvel = angvel; }

    void entity2D::mass(const float mass) { m_mass = mass; }
    void entity2D::charge(const float charge) { m_charge = charge; }

    bool operator==(const entity2D &lhs, const entity2D &rhs) { return lhs.id() == rhs.id(); }
    bool operator!=(const entity2D &lhs, const entity2D &rhs) { return lhs.id() != rhs.id(); }
}
