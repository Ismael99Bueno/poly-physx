#ifndef PPX_ENTITY2D_HPP
#define PPX_ENTITY2D_HPP

#include "geo/aabb2D.hpp"
#include "geo/polygon.hpp"
#include "geo/circle.hpp"
#include "rk/state.hpp"
#include "ppx/events/entity_events.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/interface/indexable.hpp"
#include "kit/memory/track_ptr.hpp"
#include <variant>

namespace ppx
{
class entity2D : public kit::identifiable, public kit::indexable
{
  public:
    using ptr = kit::track_ptr<entity2D>;
    using const_ptr = kit::const_track_ptr<entity2D>;
    enum class shape_type
    {
        POLYGON = 0,
        CIRCLE = 1
    };
    struct specs
    {
        glm::vec2 pos{0.f}, vel{0.f};
        float angpos = 0.f, angvel = 0.f, mass = 1.f, charge = 1.f;
        kit::block_vector<glm::vec2> vertices = geo::polygon::box(5.f);
        float radius = 2.5f;
        bool kinematic = true;
        shape_type shape = shape_type::POLYGON;
        static specs from_entity(const entity2D &e);
    };

    entity2D(const kit::block_vector<glm::vec2> &vertices, const glm::vec2 &pos = glm::vec2(0.f),
             const glm::vec2 &vel = glm::vec2(0.f), float angpos = 0.f, float angvel = 0.f, float mass = 1.f,
             float charge = 1.f, bool kinematic = true);
    entity2D(float radius, const glm::vec2 &pos = glm::vec2(0.f), const glm::vec2 &vel = glm::vec2(0.f),
             float angpos = 0.f, float angvel = 0.f, float mass = 1.f, float charge = 1.f, bool kinematic = true);
    entity2D(const glm::vec2 &pos = glm::vec2(0.f), const glm::vec2 &vel = glm::vec2(0.f), float angpos = 0.f,
             float angvel = 0.f, float mass = 1.f, float charge = 1.f, bool kinematic = true);
    entity2D(const specs &spc);

    void retrieve();
    void dispatch() const;
    float kinetic_energy() const;

    void add_force(const glm::vec2 &force);
    void add_torque(float torque);

    const glm::vec2 &added_force() const;
    float added_torque() const;

    const geo::shape2D &shape() const;

    template <typename T> const T &shape() const;

    template <typename T> const T *shape_if() const;

    void shape(const kit::block_vector<glm::vec2> &vertices);
    void shape(float radius);
    void shape(const geo::polygon &poly);
    void shape(const geo::circle &c);

    shape_type type() const;

    float inertia() const;
    float inverse_inertia() const;

    bool kinematic() const;
    void kinematic(bool kinematic);

    void translate(const glm::vec2 &dpos);
    void rotate(float dangle);

    const entity_events &events() const;
    entity_events &events();

    const glm::vec2 &pos() const;
    const glm::vec2 &vel() const;
    const glm::vec2 vel_at(const glm::vec2 &at) const;
    float angpos() const;
    float angvel() const;
    float mass() const;
    float inverse_mass() const;
    float charge() const;

    void pos(const glm::vec2 &pos);
    void vel(const glm::vec2 &vel);
    void angpos(float angpos);
    void angvel(float angvel);
    void mass(float mass);
    void charge(float charge);

  private:
    std::variant<geo::polygon, geo::circle> m_shape;
    rk::state *m_state = nullptr;
    glm::vec2 m_vel{0.f}, m_added_force{0.f};
    entity_events m_events;
    float m_angvel, m_added_torque = 0.f, m_mass, m_inv_mass, m_inertia, m_inv_inertia, m_charge;
    bool m_kinematic;

    geo::shape2D &get_shape();
    void retrieve(const std::vector<float> &vars_buffer);
    void compute_inertia(const geo::shape2D &sh);

    friend class engine2D;
};
#ifdef YAML_CPP_COMPAT
YAML::Emitter &operator<<(YAML::Emitter &out, const entity2D &e);
#endif
} // namespace ppx

#ifdef YAML_CPP_COMPAT
namespace YAML
{
template <> struct convert<ppx::entity2D>
{
    static Node encode(const ppx::entity2D &e);
    static bool decode(const Node &node, ppx::entity2D &e);
};
} // namespace YAML
#endif

#endif