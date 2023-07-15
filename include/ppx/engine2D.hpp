#ifndef PPX_ENGINE2D_HPP
#define PPX_ENGINE2D_HPP

#include "rk/integrator.hpp"
#include "ppx/entity2D_ptr.hpp"
#include "ppx/constraints/compeller2D.hpp"
#include "ppx/collision/collider2D.hpp"
#include "ppx/joints/spring2D.hpp"
#include "ppx/events/engine_events.hpp"
#include "kit/container/container_view.hpp"
#include "rk/tableaus.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class behaviour2D;
class engine2D : kit::non_copyable
{
  public:
    engine2D(const rk::butcher_tableau &table = rk::rk4, std::size_t allocations = 100);

    void retrieve();

    bool raw_forward(float &timestep);
    bool reiterative_forward(float &timestep, std::uint8_t reiterations = 2);
    bool embedded_forward(float &timestep);

    template <class... EntityArgs> entity2D_ptr add_entity(EntityArgs &&...args)
    {
        entity2D &e = m_entities.emplace_back(std::forward<EntityArgs>(args)...);
        return process_entity_addition(e);
    }

    bool remove_entity(std::size_t index);
    bool remove_entity(const entity2D &e);

    template <typename T, class... BehaviourArgs> T *add_behaviour(BehaviourArgs &&...args)
    {
        static_assert(std::is_base_of<behaviour2D, T>::value, "Type must inherit from behaviour2D! (Although it is "
                                                              "recommended to inherit from force2D or interaction2D)");
        auto bhv = kit::make_scope<T>(std::forward<BehaviourArgs>(args)...);
        T *ptr = bhv.get();

        m_behaviours.push_back(std::move(bhv));
        m_behaviours.back()->m_entities = &m_entities;
        m_events.on_behaviour_addition(*ptr);
        return ptr;
    }

    template <class... SpringArgs> spring2D &add_spring(SpringArgs &&...args)
    {
        spring2D &sp = m_springs.emplace_back(std::forward<SpringArgs>(args)...);
        m_events.on_spring_addition(&sp);
        return sp;
    }

    bool remove_behaviour(const behaviour2D *bhv);
    bool remove_spring(std::size_t index);
    bool remove_spring(const spring2D &sp);

    void clear_entities();
    void clear_behaviours();
    void clear_springs();
    void clear_constraints();
    void clear();

    void checkpoint();
    void revert();

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    std::vector<float> operator()(float t, float dt, const std::vector<float> &vars);

    const_entity2D_ptr from_id(kit::uuid id) const;
    entity2D_ptr from_id(kit::uuid id);

    template <typename T> T *behaviour_from_name(const char *name) const
    {
        static_assert(std::is_base_of<behaviour2D, T>::value, "Type must inherit from behaviour2D! (Although it is "
                                                              "recommended to inherit from force2D or interaction2D)");
        return dynamic_cast<T *>(behaviour_from_name<behaviour2D>(name));
    }

    const_entity2D_ptr operator[](std::size_t index) const;
    entity2D_ptr operator[](std::size_t index);

    std::vector<const_entity2D_ptr> operator[](const geo::aabb2D &aabb) const;
    std::vector<entity2D_ptr> operator[](const geo::aabb2D &aabb);

    const_entity2D_ptr operator[](const glm::vec2 &point) const;
    entity2D_ptr operator[](const glm::vec2 &point);

    const std::vector<kit::scope<behaviour2D>> &behaviours() const;
    const std::vector<spring2D> &springs() const;

    kit::vector<kit::scope<behaviour2D>> behaviours();
    kit::vector<spring2D> springs();

    const std::vector<entity2D> &entities() const;
    kit::vector<entity2D> entities();
    std::size_t size() const;

    const rk::integrator &integrator() const;
    rk::integrator &integrator();

    const collider2D &collider() const;
    collider2D &collider();

    const compeller2D &compeller() const;
    compeller2D &compeller();

    engine_events &events();

    float elapsed() const;

  private:
    std::vector<entity2D> m_entities;
    collider2D m_collider;
    compeller2D m_compeller;
    std::vector<kit::scope<behaviour2D>> m_behaviours;
    std::vector<spring2D> m_springs;
    std::tuple<float, std::vector<float>, std::vector<entity2D>> m_checkpoint;
    rk::integrator m_integ;
    engine_events m_events;

    float m_elapsed = 0.f;

    entity2D_ptr process_entity_addition(entity2D &e);

    void load_velocities_and_added_forces(std::vector<float> &stchanges) const;
    void load_interactions_and_externals(std::vector<float> &stchanges) const;

    kit::stack_vector<float> effective_inverse_masses() const;

    void reset_entities();
    void retrieve(const std::vector<float> &vars_buffer);
    void validate();
    std::optional<std::size_t> index_from_id(kit::uuid id) const;

#ifdef YAML_CPP_COMPAT
    friend YAML::Emitter &operator<<(YAML::Emitter &, const engine2D &);
    friend struct YAML::convert<engine2D>;
#endif
};
#ifdef YAML_CPP_COMPAT
YAML::Emitter &operator<<(YAML::Emitter &out, const engine2D &eng);
#endif
} // namespace ppx

#ifdef YAML_CPP_COMPAT
namespace YAML
{
template <> struct convert<ppx::engine2D>
{
    static Node encode(const ppx::engine2D &eng);
    static bool decode(const Node &node, ppx::engine2D &eng);
};
} // namespace YAML
#endif

#endif