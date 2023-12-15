#ifndef PPX_SPRING_MANAGER2D_HPP
#define PPX_SPRING_MANAGER2D_HPP

#include "ppx/joints/spring2D.hpp"

namespace ppx
{
class world2D;

class spring_manager2D
{
  public:
    spring_manager2D(world2D &world);

    // Move to process addtition
    template <class... SpringArgs> spring2D::ptr add(SpringArgs &&...args)
    {
        spring2D &sp = m_springs.emplace_back(std::forward<SpringArgs>(args)...);
        return process_addition(sp);
    }

    auto begin() const
    {
        return m_springs.begin();
    }
    auto end() const
    {
        return m_springs.end();
    }

    auto begin()
    {
        return m_springs.begin();
    }
    auto end()
    {
        return m_springs.end();
    }

    const spring2D &operator[](std::size_t index) const;
    spring2D &operator[](std::size_t index);

    spring2D::const_ptr operator[](kit::uuid id) const;
    spring2D::ptr operator[](kit::uuid id);

    std::vector<spring2D::const_ptr> from_ids(kit::uuid id1, kit::uuid id2) const;
    std::vector<spring2D::ptr> from_ids(kit::uuid id1, kit::uuid id2);

    spring2D::const_ptr ptr(std::size_t index) const;
    spring2D::ptr ptr(std::size_t index);

    bool remove(std::size_t index);
    bool remove(const spring2D &body);
    bool remove(kit::uuid id);

    void apply_forces();

    std::size_t size() const;
    void clear();
    void validate();

  private:
    world2D &m_world;
    std::vector<spring2D> m_springs;

    spring2D::ptr process_addition(spring2D &sp);
};
} // namespace ppx

#endif