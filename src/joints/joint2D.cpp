#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
joint2D::joint2D(world2D &world, const specs::joint2D &spc, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2)
    : joint2D(world, spc.bindex1 != SIZE_MAX ? world.bodies[spc.bindex1] : world.bodies.add(spc.bspecs1),
              spc.bindex2 != SIZE_MAX ? world.bodies[spc.bindex2] : world.bodies.add(spc.bspecs2), ganchor1, ganchor2,
              spc.bodies_collide)
{
}

joint2D::joint2D(world2D &world, const specs::joint2D &spc, const glm::vec2 &ganchor)
    : joint2D(world, spc, ganchor, ganchor)
{
}

joint2D::joint2D(world2D &world, const specs::joint2D &spc)
    : joint2D(world, spc.bindex1 != SIZE_MAX ? world.bodies[spc.bindex1] : world.bodies.add(spc.bspecs1),
              spc.bindex2 != SIZE_MAX ? world.bodies[spc.bindex2] : world.bodies.add(spc.bspecs2), spc.bodies_collide)
{
}

joint2D::joint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2,
                 const bool bodies_collide)
    : worldref2D(world), bodies_collide(bodies_collide), m_body1(body1), m_body2(body2),
      m_lanchor1(body1->local_position_point(ganchor1)), m_lanchor2(body2->local_position_point(ganchor2)),
      m_no_anchors(false)
{
    KIT_ASSERT_ERROR(body1 != body2, "Cannot create joint between the same body: {0}", body1->index);
}

joint2D::joint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor, const bool bodies_collide)
    : joint2D(world, body1, body2, ganchor, ganchor)
{
}

joint2D::joint2D(world2D &world, body2D *body1, body2D *body2, const bool bodies_collide)
    : worldref2D(world), bodies_collide(bodies_collide), m_body1(body1), m_body2(body2), m_lanchor1(0.f),
      m_lanchor2(0.f), m_no_anchors(true)
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
body2D *joint2D::body1()
{
    return m_body1;
}
body2D *joint2D::body2()
{
    return m_body2;
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
    return m_no_anchors ? m_body1->centroid() : m_body1->global_position_point(m_lanchor1);
}
glm::vec2 joint2D::ganchor2() const
{
    return m_no_anchors ? m_body2->centroid() : m_body2->global_position_point(m_lanchor2);
}

bool joint2D::contains(const body2D *body) const
{
    return body == m_body1 || body == m_body2;
}
bool joint2D::is_constraint() const
{
    return false;
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