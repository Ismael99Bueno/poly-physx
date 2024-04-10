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
    m_single_anchor = false;
}

joint2D::joint2D(world2D &world, const specs::joint2D &spc, const glm::vec2 &ganchor)
    : joint2D(world, spc, ganchor, ganchor)
{
    m_single_anchor = true;
}

joint2D::joint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2,
                 const bool bodies_collide)
    : worldref2D(world), bodies_collide(bodies_collide), m_body1(body1), m_body2(body2),
      m_lanchor1(body1->local_position_point(ganchor1)), m_lanchor2(body2->local_position_point(ganchor2))
{
    KIT_ASSERT_ERROR(body1 != body2, "Cannot create joint between the same body: {0}", body1->index);
    m_single_anchor = false;
}

joint2D::joint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor, const bool bodies_collide)
    : joint2D(world, body1, body2, ganchor, ganchor)
{
    m_single_anchor = true;
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

bool joint2D::single_anchor() const
{
    return m_single_anchor;
}

const glm::vec2 &joint2D::lanchor1() const
{
    return m_lanchor1;
}
const glm::vec2 &joint2D::lanchor2() const
{
    return m_lanchor2;
}

glm::vec2 joint2D::ganchor() const
{
    KIT_ASSERT_WARN(m_single_anchor, "Joint has two anchors, you should use ganchor1 or ganchor2 instead")
    return m_body1->global_position_point(m_lanchor1);
}
glm::vec2 joint2D::ganchor1() const
{
    KIT_ASSERT_WARN(!m_single_anchor, "Joint has one anchor, you should use ganchor instead")
    return m_body1->global_position_point(m_lanchor1);
}
glm::vec2 joint2D::ganchor2() const
{
    KIT_ASSERT_WARN(!m_single_anchor, "Joint has one anchor, you should use ganchor instead")
    return m_body2->global_position_point(m_lanchor2);
}

bool joint2D::contains(const body2D *body) const
{
    return body == m_body1 || body == m_body2;
}

void joint2D::add_to_bodies()
{
    m_body1->m_joints.push_back(this);
    m_body2->m_joints.push_back(this);
}

void joint2D::remove_from_bodies()
{
    for (auto it = m_body1->m_joints.begin(); it != m_body1->m_joints.end(); ++it)
        if (*it == this)
        {
            m_body1->m_joints.erase(it);
            break;
        }
    for (auto it = m_body2->m_joints.begin(); it != m_body2->m_joints.end(); ++it)
        if (*it == this)
        {
            m_body2->m_joints.erase(it);
            break;
        }
}

} // namespace ppx