#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint_manager2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
ijoint_manager2D::ijoint_manager2D(const std::string &name) : kit::identifiable<std::string>(name)
{
}

} // namespace ppx