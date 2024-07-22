#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
joint2D::joint2D(world2D &world, const specs::joint2D &spc, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2,
                 const specs::joint2D::properties &jprops)
    : joint2D(world, spc.bindex1 != SIZE_MAX ? world.bodies[spc.bindex1] : world.bodies.add(spc.bspecs1),
              spc.bindex2 != SIZE_MAX ? world.bodies[spc.bindex2] : world.bodies.add(spc.bspecs2), ganchor1, ganchor2,
              jprops)
{
}

joint2D::joint2D(world2D &world, const specs::joint2D &spc, const glm::vec2 &ganchor,
                 const specs::joint2D::properties &jprops)
    : joint2D(world, spc, ganchor, ganchor, jprops)
{
}

joint2D::joint2D(world2D &world, const specs::joint2D &spc, const specs::joint2D::properties &jprops)
    : joint2D(world, spc.bindex1 != SIZE_MAX ? world.bodies[spc.bindex1] : world.bodies.add(spc.bspecs1),
              spc.bindex2 != SIZE_MAX ? world.bodies[spc.bindex2] : world.bodies.add(spc.bspecs2), jprops)
{
}

joint2D::joint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2,
                 const specs::joint2D::properties &jprops)
    : worldref2D(world), m_body1(body1), m_body2(body2), m_lanchor1(body1->state().local_position_point(ganchor1)),
      m_lanchor2(body2->state().local_position_point(ganchor2)), m_bodies_collide(jprops.bodies_collide)
{
    KIT_ASSERT_ERROR(body1 != body2, "Cannot create joint between the same body: {0}", body1->meta.index);
}

joint2D::joint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor,
                 const specs::joint2D::properties &jprops)
    : joint2D(world, body1, body2, ganchor, ganchor, jprops)
{
}

joint2D::joint2D(world2D &world, body2D *body1, body2D *body2, const specs::joint2D::properties &jprops)
    : worldref2D(world), m_body1(body1), m_body2(body2), m_lanchor1(0.f), m_lanchor2(0.f),
      m_bodies_collide(jprops.bodies_collide)
{
}

const body2D *joint2D::body1() const
{
    return m_body1;
}
const body2D *joint2D::body2() const
{
    return m_body2;
}
const body2D *joint2D::other(const body2D *body) const
{
    return body == m_body1 ? m_body2 : m_body1;
}

body2D *joint2D::body1()
{
    return m_body1;
}
body2D *joint2D::body2()
{
    return m_body2;
}
body2D *joint2D::other(const body2D *body)
{
    return body == m_body1 ? m_body2 : m_body1;
}

const glm::vec2 &joint2D::lanchor1() const
{
    return m_lanchor1;
}
const glm::vec2 &joint2D::lanchor2() const
{
    return m_lanchor2;
}

glm::vec2 joint2D::ganchor1() const
{
    return m_body1->state().global_position_point(m_lanchor1);
}
glm::vec2 joint2D::ganchor2() const
{
    return m_body2->state().global_position_point(m_lanchor2);
}

bool joint2D::contains(const body2D *body) const
{
    return body == m_body1 || body == m_body2;
}

void joint2D::awake()
{
    m_body1->awake();
    m_body2->awake();
}
bool joint2D::asleep() const
{
    return m_body1->asleep() && m_body2->asleep();
}

bool joint2D::bodies_collide() const
{
    return m_bodies_collide;
}
void joint2D::bodies_collide(const bool bodies_collide)
{
    m_bodies_collide = bodies_collide;
    awake();
}

specs::joint2D::properties joint2D::jprops() const
{
    return {m_bodies_collide};
}
void joint2D::jprops(const specs::joint2D::properties &jprops)
{
    m_bodies_collide = jprops.bodies_collide;
    awake();
}

void joint2D::fill_jprops(specs::joint2D::properties &jprops) const
{
    jprops.bodies_collide = m_bodies_collide;
}

bool joint2D::is_constraint() const
{
    return false;
}
bool joint2D::is_actuator() const
{
    return false;
}
bool joint2D::is_contact() const
{
    return false;
}

void joint2D::compute_anchors_and_offsets(const state2D &state1, const state2D &state2)
{
    m_ganchor1 = state1.global_position_point(m_lanchor1);
    m_ganchor2 = state2.global_position_point(m_lanchor2);

    m_offset1 = m_ganchor1 - state1.centroid.position;
    m_offset2 = m_ganchor2 - state2.centroid.position;
}

void joint2D::add_to_bodies()
{
    m_body1->meta.joints.push_back(this);
    m_body2->meta.joints.push_back(this);
}

void joint2D::remove_from_bodies()
{
    m_body1->meta.remove_joint(this);
    m_body2->meta.remove_joint(this);
}

} // namespace ppx