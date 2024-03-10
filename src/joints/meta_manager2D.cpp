#include "ppx/internal/pch.hpp"
#include "ppx/joints/meta_manager2D.hpp"
#include "ppx/joints/joint_manager2D.hpp"
#include "ppx/constraints/constraint_manager2D.hpp"

namespace ppx
{
template <Solver S> bool meta_manager2D<S>::remove(std::size_t index)
{
    if (index >= this->m_elements.size())
        return false;
    this->m_elements.erase(this->m_elements.begin() + index);
    return true;
}

template class meta_manager2D<joint_solver2D>;
template class meta_manager2D<constraint_solver2D>;
} // namespace ppx