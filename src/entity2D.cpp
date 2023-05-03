#include "ppx/pch.hpp"
#include "ppx/entity2D.hpp"
#include "ppx/force2D.hpp"
#include "ppx/interaction2D.hpp"
#include "debug/debug.hpp"

namespace ppx
{
    entity2D::entity2D(const std::variant<geo::polygon, geo::circle> &shape,
                       const glm::vec2 &pos, const glm::vec2 &vel,
                       const float angpos, const float angvel,
                       const float mass, const float charge,
                       const bool kinematic) : m_shape(shape),
                                               m_vel(vel),
                                               m_angvel(angvel),
                                               m_mass(mass),
                                               m_charge(charge),
                                               m_kinematic(kinematic)
    {
        geo::shape2D &sh = get_shape();
        sh.pos(pos);
        sh.rotation(angpos);
    }

    entity2D::entity2D(const glm::vec2 &pos, const glm::vec2 &vel,
                       const float angpos, const float angvel,
                       const float mass, const float charge,
                       const bool kinematic) : m_shape(geo::polygon(pos, angpos, geo::polygon::box(5.f))),
                                               m_vel(vel),
                                               m_angvel(angvel),
                                               m_mass(mass),
                                               m_charge(charge),
                                               m_kinematic(kinematic) {}
    entity2D::entity2D(const std::vector<glm::vec2> &vertices,
                       const glm::vec2 &pos, const glm::vec2 &vel,
                       const float angpos, const float angvel,
                       const float mass, const float charge,
                       const bool kinematic) : m_shape(geo::polygon(pos, angpos, vertices)),
                                               m_vel(vel),
                                               m_angvel(angvel),
                                               m_mass(mass),
                                               m_charge(charge),
                                               m_kinematic(kinematic) {}
    entity2D::entity2D(const float radius,
                       const glm::vec2 &pos,
                       const glm::vec2 &vel,
                       const float angpos, const float angvel,
                       const float mass, const float charge,
                       const bool kinematic) : m_shape(geo::circle(pos, radius, angpos)),
                                               m_vel(vel),
                                               m_angvel(angvel),
                                               m_mass(mass),
                                               m_charge(charge),
                                               m_kinematic(kinematic) {}

    void entity2D::retrieve(const std::vector<float> &vars_buffer)
    {
        const std::size_t idx = 6 * m_index;
        geo::shape2D &sh = get_shape();

        sh.pos({vars_buffer[idx + 0], vars_buffer[idx + 1]});
        sh.rotation(vars_buffer[idx + 2]);

        m_vel = {vars_buffer[idx + 3], vars_buffer[idx + 4]};
        m_angvel = vars_buffer[idx + 5];
    }

    void entity2D::retrieve()
    {
        DBG_ASSERT(m_state, "Cannot retrieve from a null state.\n")
        retrieve(m_state->vars());
    }
    void entity2D::dispatch() const
    {
        const geo::shape2D &sh = shape();
        const glm::vec2 &pos = sh.centroid();
        const float angpos = sh.rotation();
        rk::state &st = *m_state;

        const std::size_t idx = 6 * m_index;
        st[idx + 0] = pos.x;
        st[idx + 1] = pos.y;
        st[idx + 2] = angpos;
        st[idx + 3] = m_vel.x;
        st[idx + 4] = m_vel.y;
        st[idx + 5] = m_angvel;
    }

    float entity2D::kinetic_energy() const
    {
        return 0.5f * (m_mass * glm::length2(m_vel) + m_angvel * m_angvel * shape().inertia());
    }

    void entity2D::add_force(const glm::vec2 &force) { m_added_force += force; }
    void entity2D::add_torque(const float torque) { m_added_torque += torque; }

    const glm::vec2 &entity2D::force() const { return m_force; }
    float entity2D::torque() const { return m_torque; }
    const glm::vec2 &entity2D::added_force() const { return m_added_force; }
    float entity2D::added_torque() const { return m_added_torque; }

    const geo::shape2D &entity2D::shape() const
    {
        if (m_shape.index() == 0)
            return std::get<geo::polygon>(m_shape);
        return std::get<geo::circle>(m_shape);
    }
    geo::shape2D &entity2D::get_shape()
    {
        if (m_shape.index() == 0)
            return std::get<geo::polygon>(m_shape);
        return std::get<geo::circle>(m_shape);
    }
    template <typename T>
    const T &entity2D::shape() const { return std::get<T>(m_shape); }

    template const geo::polygon &entity2D::shape<geo::polygon>() const;
    template const geo::circle &entity2D::shape<geo::circle>() const;

    template <typename T>
    const T *entity2D::shape_if() const { return std::get_if<T>(&m_shape); }

    template const geo::polygon *entity2D::shape_if<geo::polygon>() const;
    template const geo::circle *entity2D::shape_if<geo::circle>() const;

    void entity2D::shape(const std::vector<glm::vec2> &vertices)
    {
        const geo::shape2D &sh = shape();
        m_shape = geo::polygon(sh.centroid(), sh.rotation(), vertices);
    }
    void entity2D::shape(const float radius)
    {
        const geo::shape2D &sh = shape();
        m_shape = geo::circle(sh.centroid(), radius, sh.rotation());
    }
    void entity2D::shape(const geo::polygon &poly) { m_shape = poly; }
    void entity2D::shape(const geo::circle &c) { m_shape = c; }

    entity2D::shape_type entity2D::type() const { return m_shape.index() == 0 ? POLYGON : CIRCLE; }

    std::size_t entity2D::index() const { return m_index; }
    std::size_t entity2D::id() const { return m_id; }

    float entity2D::inertia() const { return shape().inertia() * m_mass; }

    bool entity2D::kinematic() const { return m_kinematic; }
    void entity2D::kinematic(const bool kinematic) { m_kinematic = kinematic; }

    void entity2D::translate(const glm::vec2 &dpos) { get_shape().translate(dpos); }
    void entity2D::rotate(const float dangle) { get_shape().rotate(dangle); }

    const entity_events &entity2D::events() const { return m_events; }
    entity_events &entity2D::events() { return m_events; }

    const glm::vec2 &entity2D::pos() const { return shape().centroid(); }
    const glm::vec2 &entity2D::vel() const { return m_vel; }
    const glm::vec2 entity2D::vel_at(const glm::vec2 &at) const { return m_vel + m_angvel * glm::vec2(-at.y, at.x); }

    float entity2D::angpos() const { return shape().rotation(); }
    float entity2D::angvel() const { return m_angvel; }

    float entity2D::mass() const { return m_mass; }
    float entity2D::charge() const { return m_charge; }

    void entity2D::pos(const glm::vec2 &pos) { get_shape().pos(pos); }
    void entity2D::vel(const glm::vec2 &vel) { m_vel = vel; }

    void entity2D::angpos(const float angpos) { get_shape().rotation(angpos); }
    void entity2D::angvel(const float angvel) { m_angvel = angvel; }

    void entity2D::mass(const float mass) { m_mass = mass; }
    void entity2D::charge(const float charge) { m_charge = charge; }

    bool operator==(const entity2D &lhs, const entity2D &rhs) { return lhs.id() == rhs.id(); }
    bool operator!=(const entity2D &lhs, const entity2D &rhs) { return lhs.id() != rhs.id(); }
#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const entity2D &e)
    {
        out << YAML::BeginMap;
        out << YAML::Key << "id" << e.id();
        out << YAML::Key << "index" << e.index();
        out << YAML::Key << "shape" << e.shape();
        out << YAML::Key << "vel" << e.vel();
        out << YAML::Key << "angvel" << e.angvel();
        out << YAML::Key << "mass" << e.mass();
        out << YAML::Key << "charge" << e.charge();
        out << YAML::Key << "kinematic" << e.kinematic();
        out << YAML::EndMap;
        return out;
    }
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    Node convert<ppx::entity2D>::encode(const ppx::entity2D &e)
    {
        Node node;
        node["id"] = e.id();
        node["index"] = e.index();
        node["shape"] = e.shape();
        node["vel"] = e.vel();
        node["angvel"] = e.angvel();
        node["mass"] = e.mass();
        node["charge"] = e.charge();
        node["kinematic"] = e.kinematic();
        return node;
    }
    bool convert<ppx::entity2D>::decode(const Node &node, ppx::entity2D &e)
    {
        if (!node.IsMap() || node.size() != 8)
            return false;

        e.m_id = node["id"].as<std::size_t>();
        e.m_index = node["index"].as<std::size_t>();
        if (node["shape"]["radius"])
            e.shape(node["shape"].as<geo::circle>());
        else
            e.shape(node["shape"].as<geo::polygon>());
        e.vel(node["vel"].as<glm::vec2>());
        e.angvel(node["angvel"].as<float>());
        e.mass(node["mass"].as<float>());
        e.charge(node["charge"].as<float>());
        e.kinematic(node["kinematic"].as<bool>());

        return true;
    };
}
#endif
