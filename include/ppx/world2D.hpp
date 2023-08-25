#ifndef PPX_ENGINE2D_HPP
#define PPX_ENGINE2D_HPP

#include "rk/integrator.hpp"
#include "ppx/constraints/compeller2D.hpp"
#include "ppx/collision/collider2D.hpp"
#include "ppx/joints/spring2D.hpp"
#include "ppx/events/world_events.hpp"
#include "kit/container/container_view.hpp"
#include "rk/tableaus.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class behaviour2D;
class world2D final : kit::non_copyable
{
  public:
#ifdef KIT_USE_YAML_CPP
    class serializer : public kit::serializer<world2D>
    {
      public:
        YAML::Node encode(const world2D &world) const override;
        bool decode(const YAML::Node &node, world2D &world) const override;
    };
#endif

    world2D(const rk::butcher_tableau &table = rk::rk4, std::size_t allocations = 100);

    collider2D collisions;
    rk::integrator integrator;
    world_events events;

    bool raw_forward(float timestep);
    bool reiterative_forward(float &timestep, std::uint8_t reiterations = 2);
    bool embedded_forward(float &timestep);

    template <class... BodyArgs> body2D::ptr add_body(BodyArgs &&...args)
    {
        body2D &body = m_bodies.emplace_back(std::forward<BodyArgs>(args)...);
        return process_body_addition(body);
    }

    bool remove_body(std::size_t index);
    bool remove_body(const body2D &body);
    bool remove_body(kit::uuid id);

    template <typename T, class... BehaviourArgs> T *add_behaviour(BehaviourArgs &&...args)
    {
        static_assert(std::is_base_of_v<behaviour2D, T>, "Type must inherit from behaviour2D! (Although it is "
                                                         "recommended to inherit from force2D or interaction2D)");
        auto bhv = kit::make_scope<T>(std::forward<BehaviourArgs>(args)...);
#ifdef DEBUG
        for (const auto &old : m_behaviours)
        {
            KIT_ASSERT_ERROR(
                *old != *bhv,
                "Cannot add a behaviour with a name that already exists. Behaviour names act as identifiers")
        }
#endif
        T *ptr = bhv.get();

        m_behaviours.push_back(std::move(bhv));
        m_behaviours.back()->m_parent = this;
        events.on_behaviour_addition(ptr);
        return ptr;
    }

    bool remove_behaviour(std::size_t index);
    bool remove_behaviour(const behaviour2D *bhv);
    bool remove_behaviour(const std::string &name);

    template <class... SpringArgs> spring2D::ptr add_spring(SpringArgs &&...args)
    {
        spring2D &sp = m_springs.emplace_back(std::forward<SpringArgs>(args)...);
        sp.index = m_springs.size() - 1;

        const spring2D::ptr sp_ptr = {&m_springs, sp.index};
        events.on_spring_addition(sp_ptr);
        return sp_ptr;
    }

    bool remove_spring(std::size_t index);
    bool remove_spring(const spring2D &sp);
    bool remove_spring(kit::uuid id);

    template <typename T, class... ConstraintArgs> T *add_constraint(ConstraintArgs &&...args)
    {
        T *ptr = m_compeller.add_constraint<T>(events.on_constraint_addition, std::forward<ConstraintArgs>(args)...);
        return ptr;
    }

    bool remove_constraint(std::size_t index);
    bool remove_constraint(const constraint2D *ctr);
    bool remove_constraint(kit::uuid id);

    void clear_bodies();
    void clear_behaviours();
    void clear_springs();
    void clear_constraints();
    void clear();

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    std::vector<float> operator()(float t, float dt, const std::vector<float> &vars);

    body2D::const_ptr body_from_id(kit::uuid id) const;
    body2D::ptr body_from_id(kit::uuid id);

    spring2D::const_ptr spring_from_id(kit::uuid id) const;
    spring2D::ptr spring_from_id(kit::uuid id);

    const constraint2D *constraint_from_id(kit::uuid id) const;
    constraint2D *constraint_from_id(kit::uuid id);

    template <typename T> T *behaviour_from_name(const std::string &name) const
    {
        static_assert(std::is_base_of_v<behaviour2D, T>, "Type must inherit from behaviour2D! (Although it is "
                                                         "recommended to inherit from force2D or interaction2D)");
        return dynamic_cast<T *>(behaviour_from_name<behaviour2D>(name));
    }

    body2D::const_ptr operator[](std::size_t index) const;
    body2D::ptr operator[](std::size_t index);

    std::vector<body2D::const_ptr> operator[](const geo::aabb2D &aabb) const;
    std::vector<body2D::ptr> operator[](const geo::aabb2D &aabb);

    body2D::const_ptr operator[](const glm::vec2 &point) const;
    body2D::ptr operator[](const glm::vec2 &point);

    const std::vector<body2D> &bodies() const;
    const std::vector<spring2D> &springs() const;

    spring2D::const_ptr spring(std::size_t index) const;
    std::vector<spring2D::const_ptr> springs_from_ids(kit::uuid id1, kit::uuid id2) const;

    kit::vector_view<body2D> bodies();
    kit::vector_view<spring2D> springs();

    spring2D::ptr spring(std::size_t index);
    std::vector<spring2D::ptr> springs_from_ids(kit::uuid id1, kit::uuid id2);

    const std::vector<kit::scope<behaviour2D>> &behaviours() const;
    const std::vector<kit::scope<constraint2D>> &constraints() const;

    std::vector<const constraint2D *> constraints_from_ids(const std::vector<kit::uuid> &ids) const;
    std::vector<constraint2D *> constraints_from_ids(const std::vector<kit::uuid> &ids);

    std::size_t size() const;
    float elapsed() const;

  private:
    std::vector<body2D> m_bodies;
    compeller2D m_compeller;
    std::vector<kit::scope<behaviour2D>> m_behaviours;
    std::vector<spring2D> m_springs;

    float m_elapsed = 0.f;

    body2D::ptr process_body_addition(body2D &body);

    void load_velocities_and_added_forces(std::vector<float> &stchanges) const;
    void load_interactions_and_externals(std::vector<float> &stchanges) const;

    kit::stack_vector<float> effective_inverse_masses() const;

    void reset_bodies();
    void retrieve(const std::vector<float> &vars_buffer);
    void retrieve();
    void validate();
};

} // namespace ppx

#endif