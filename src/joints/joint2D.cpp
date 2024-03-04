#pragma once

#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
joint2D::joint2D(world2D &world, const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &ganchor1,
                 const glm::vec2 &ganchor2)
    : m_body1(body1), m_body2(body2), m_lanchor1(body1->local_position_point(ganchor1)),
      m_lanchor2(body2->local_position_point(ganchor2))
{
}

joint2D::joint2D(world2D &world, const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &ganchor)
    : m_body1(body1), m_body2(body2), m_lanchor1(body1->local_position_point(ganchor)),
      m_lanchor2(body2->local_position_point(ganchor))
{
}

const body2D::ptr &joint2D::body1() const
{
    return m_body1;
}
const body2D::ptr &joint2D::body2() const
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
    return m_body1->global_position_point(m_lanchor1);
}
glm::vec2 joint2D::ganchor2() const
{
    return m_body2->global_position_point(m_lanchor2);
}

glm::vec2 joint2D::ganchor_offset1() const
{
    return ganchor1() - m_body1->centroid();
}
glm::vec2 joint2D::ganchor_offset2() const
{
    return ganchor2() - m_body2->centroid();
}
std::tuple<glm::vec2, glm::vec2, glm::vec2> joint2D::dir_and_offsets() const
{
    const glm::vec2 g1 = ganchor1();
    const glm::vec2 g2 = ganchor2();
    const glm::vec2 dir = g2 - g1;
    return {dir, g1 - m_body1->centroid(), g2 - m_body2->centroid()};
}

bool joint2D::valid() const
{
    return m_body1 && m_body2;
}

} // namespace ppx