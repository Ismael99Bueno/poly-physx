#pragma once

#include "ppx/body2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/interface/nameable.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/serialization/yaml/codec.hpp"

namespace ppx
{
class world2D;
class behaviour2D : kit::non_copyable,
                    public kit::identifiable<std::string>,
                    public kit::toggleable,
                    public kit::yaml::codecable
{
  public:
    behaviour2D(const std::string &name, std::size_t allocations = 50);
    virtual ~behaviour2D() = default;

    world2D *world = nullptr;

    void validate();

    void add(const body2D::ptr &body);
    void remove(const body2D &body);
    bool contains(const body2D &body) const;

    auto begin() const
    {
        return m_bodies.begin();
    }
    auto end() const
    {
        return m_bodies.end();
    }

    auto begin()
    {
        return m_bodies.begin();
    }
    auto end()
    {
        return m_bodies.end();
    }

    body2D::const_ptr operator[](std::size_t index) const;
    const body2D::ptr &operator[](std::size_t index);

    body2D::const_ptr operator[](kit::uuid id) const;
    body2D::ptr operator[](kit::uuid id);

    void apply_force_to_bodies();

    virtual glm::vec3 force(const body2D &body) const = 0;

    float kinetic_energy() const;
    virtual float potential_energy() const = 0;
    virtual float potential_energy(const body2D &body) const = 0;

    float energy(const body2D &body) const;
    float energy() const;

    void clear();
    std::size_t size() const;

#ifdef KIT_USE_YAML_CPP
    virtual YAML::Node encode() const;
    virtual bool decode(const YAML::Node &node);
#endif

  protected:
    std::vector<body2D::ptr> m_bodies;
};

} // namespace ppx
