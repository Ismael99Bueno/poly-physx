#pragma once

#include "ppx/internal/worldref.hpp"
#include "ppx/common/specs2D.hpp"
#include "kit/interface/toggleable.hpp"

namespace ppx
{
class joint2D;

template <typename T>
concept IJoint2D = kit::DerivedFrom<T, joint2D>;

template <typename T>
concept Joint2D = requires() {
    requires IJoint2D<T>;
    typename T::specs;
    requires kit::DerivedFrom<typename T::specs, specs::joint2D>;
};

class body2D;
class joint2D : public kit::toggleable, public worldref2D
{
  public:
    virtual ~joint2D() = default;

    struct metadata
    {
        std::size_t index;
        bool island_flag = false;
    } meta;

    const body2D *body1() const;
    const body2D *body2() const;
    const body2D *other(const body2D *body) const;

    body2D *body1();
    body2D *body2();
    body2D *other(const body2D *body);

    const glm::vec2 &lanchor1() const;
    const glm::vec2 &lanchor2() const;

    glm::vec2 ganchor1() const;
    glm::vec2 ganchor2() const;

    const glm::vec2 &reactive_force() const;
    float reactive_torque() const;

    bool contains(const body2D *body) const;
    void awake();
    bool asleep() const;

    bool bodies_collide() const;
    void bodies_collide(bool bodies_collide);

    specs::joint2D::properties jprops() const;
    void jprops(const specs::joint2D::properties &jprops);

    KIT_TOGGLEABLE_FINAL_DEFAULT_SETTER()

    virtual bool is_constraint() const;
    virtual bool is_actuator() const;
    virtual bool is_contact() const;

  protected:
    joint2D(world2D &world, const specs::joint2D &spc, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2,
            const specs::joint2D::properties &jprops = {});
    joint2D(world2D &world, const specs::joint2D &spc, const glm::vec2 &ganchor,
            const specs::joint2D::properties &jprops = {});
    joint2D(world2D &world, const specs::joint2D &spc, const specs::joint2D::properties &jprops = {});

    joint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2,
            const specs::joint2D::properties &jprops = {});
    joint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor,
            const specs::joint2D::properties &jprops = {});
    joint2D(world2D &world, body2D *body1, body2D *body2, const specs::joint2D::properties &jprops = {});

    void fill_jprops(specs::joint2D::properties &jprops) const;

    body2D *m_body1;
    body2D *m_body2;

    glm::vec2 m_lanchor1;
    glm::vec2 m_lanchor2;

    glm::vec2 m_force{0.f};
    float m_torque = 0.f;

    bool m_no_anchors = false; // joint is not anchored (e.g. rotor or motor)
    bool m_use_both_anchors = true;
    bool m_bodies_collide;

    glm::vec2 m_ganchor1;
    glm::vec2 m_ganchor2;

    glm::vec2 m_offset1;
    glm::vec2 m_offset2;

    void compute_anchors_and_offsets();

  private:
    void add_to_bodies();
    void remove_from_bodies();

    template <Joint2D T> friend class joint_manager2D;
};

} // namespace ppx