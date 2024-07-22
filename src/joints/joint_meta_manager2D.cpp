#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint_meta_manager2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
template <IManager IM> bool joint_meta_manager2D<IM>::remove(std::size_t index)
{
    if (index >= this->m_elements.size())
        return false;
    this->m_elements.erase(this->m_elements.begin() + index);
    return true;
}

template <IManager IM> bool joint_meta_manager2D<IM>::remove(joint2D *joint)
{
    for (const auto &manager : this->m_elements)
        if (manager->remove(joint))
            return true;
    return false;
}

template class joint_meta_manager2D<iactuator_manager2D>;
template class joint_meta_manager2D<iconstraint_manager2D>;

} // namespace ppx