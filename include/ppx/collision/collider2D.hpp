#ifndef PPX_COLLIDER_HPP
#define PPX_COLLIDER_HPP

#include "ppx/body2D.hpp"
#include "ppx/constraints/constraint2D.hpp"
#include "ppx/collision/quad_tree2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/interface/serialization.hpp"

#include <vector>
#include <utility>

#ifndef PPX_THREAD_COUNT
#define PPX_THREAD_COUNT 8
#endif

namespace ppx
{
class world2D;
struct collision2D
{
    body2D::ptr current, incoming;
    glm::vec2 touch1{0.f}, touch2{0.f}, normal{0.f};
};

class collider2D final : kit::non_copyable, public kit::toggleable
{
  public:
#ifdef KIT_USE_YAML_CPP
    class serializer : public kit::serializer<collider2D>
    {
      public:
        YAML::Node encode(const collider2D &tb) const override;
        bool decode(const YAML::Node &node, collider2D &tb) const override;
    };
#endif

    enum class detection
    {
        BRUTE_FORCE = 0,
        SORT_AND_SWEEP = 1,
        QUAD_TREE = 2
    };

    collider2D(world2D &parent, std::size_t allocations);

    void add_body_intervals(const body2D::const_ptr &body);
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

        interval(const body2D::const_ptr &body, end end_type);

        const body2D *body() const;
        float value() const;
        end type() const;
        bool valid() const;

      private:
        body2D::const_ptr m_body;
        end m_end;
    };
    using colpair = std::pair<const body2D *, const body2D *>; // Should only last for 1 frame

    world2D &m_parent;
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

    bool narrow_detection(const body2D &body1, const body2D &body2, collision2D *c) const;
    bool narrow_detection_mix(const body2D &body1, const body2D &body2, collision2D *c) const;
    bool narrow_detection_circle(const body2D &body1, const body2D &body2, collision2D *c) const;
    bool full_detection(const body2D &body1, const body2D &body2, collision2D *c) const;

    void try_enter_or_stay_callback(const body2D &body1, const body2D &body2, const collision2D &c) const;
    void try_exit_callback(const body2D &body1, const body2D &body2) const;

    void broad_and_narrow_fase(std::vector<float> &stchanges);
    void narrow_fase(std::vector<float> &stchanges);

    void brute_force(std::vector<float> &stchanges);
    void sort_and_sweep(std::vector<float> &stchanges);
    void quad_tree(std::vector<float> &stchanges);

    void solve(const collision2D &c, std::vector<float> &stchanges) const;
    std::array<float, 6> forces_upon_collision(const collision2D &c) const;
};
} // namespace ppx

#endif