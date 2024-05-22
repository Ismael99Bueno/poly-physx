#include "ppx/internal/pch.hpp"
#include "ppx/actuators/actuator2D.hpp"

namespace ppx
{
bool actuator2D::is_constraint() const
{
    return false;
}
bool actuator2D::is_actuator() const
{
    return true;
}
} // namespace ppx