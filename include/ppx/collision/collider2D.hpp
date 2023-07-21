#ifndef PPX_COLLIDER_HPP
#define PPX_COLLIDER_HPP

#include "ppx/entity2D.hpp"
#include "ppx/constraints/constraint2D.hpp"
#include "ppx/collision/quad_tree2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/interface/toggleable.hpp"
#include <vector>
#include <utility>

#ifndef PPX_THREAD_COUNT
#define PPX_THREAD_COUNT 8
#endif

namespace ppx
{
struct collision2D
{
    entity2D::ptr current, incoming;
    glm::vec2 touch1{0.f}, touch2{0.f}, normal{0.f};
};

class collider2D final : kit::non_copyable, public kit::toggleable
{
  public:
    enum class detection
    {
        BRUTE_FORCE = 0,
        SORT_AND_SWEEP = 1,
        QUAD_TREE = 2
    };

    collider2D(kit::track_vector<entity2D> *entities, std::size_t allocations);

    void add_entity_intervals(const entity2D::const_ptr &e);
    void solve_and_load_collisions(std::vector<float> &stchanges);
    void validate();
    void flush_collisions();

    float stiffness() const;
    float dampening() const;

    void stiffness(float stiffness);
    void dampening(float dampening);

    detection detection_method() const;
    void detection_method(detection coldet);

    const quad_tree2D &quad_tree() const;
    quad_tree2D &quad_tree();

  private:
    struct interval
    {
      public:
        enum class end
        {
            LOWER,
            HIGHER
        };

        interval(const entity2D::const_ptr &e, end end_type);

        const entity2D *entity() const;
        float value() const;
        end type() const;
        bool valid() const;

      private:
        entity2D::const_ptr m_entity;
        end m_end;
    };
    using colpair = std::pair<const entity2D *, const entity2D *>; // Should only last for 1 frame

    kit::track_vector<entity2D> *m_entities;
    std::vector<interval> m_intervals;
    std::vector<colpair> m_collision_pairs;
#ifdef PPX_MULTITHREADED
    std::array<std::vector<colpair>, PPX_THREAD_COUNT> m_mt_collision_pairs;
#endif

    quad_tree2D m_quad_tree;
    float m_stiffness = 5000.f, m_dampening = 10.f;
    detection m_coldet_method = detection::QUAD_TREE;

    void sort_intervals();
    void update_quad_tree();

    bool narrow_detection(const entity2D &e1, const entity2D &e2, collision2D *c) const;
    bool narrow_detection_mix(const entity2D &e1, const entity2D &e2, collision2D *c) const;
    bool narrow_detection_circle(const entity2D &e1, const entity2D &e2, collision2D *c) const;
    bool full_detection(const entity2D &e1, const entity2D &e2, collision2D *c) const;

    void try_enter_or_stay_callback(const entity2D &e1, const entity2D &e2, const collision2D &c) const;
    void try_exit_callback(const entity2D &e1, const entity2D &e2) const;

    void broad_and_narrow_fase(std::vector<float> &stchanges);
    void narrow_fase(std::vector<float> &stchanges);

    void brute_force(std::vector<float> &stchanges);
    void sort_and_sweep(std::vector<float> &stchanges);
    void quad_tree(std::vector<float> &stchanges);

    void solve(const collision2D &c, std::vector<float> &stchanges) const;
    std::array<float, 6> forces_upon_collision(const collision2D &c) const;
};

#ifdef KIT_USE_YAML_CPP
YAML::Emitter &operator<<(YAML::Emitter &out, const collider2D &cld);
#endif
} // namespace ppx

#ifdef KIT_USE_YAML_CPP
namespace YAML
{
template <> struct convert<ppx::collider2D>
{
    static Node encode(const ppx::collider2D &cld);
    static bool decode(const Node &node, ppx::collider2D &cld);
};
} // namespace YAML
#endif

#endif