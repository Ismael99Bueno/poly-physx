#pragma once

#include "ppx/body/body2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/serialization/yaml/codec.hpp"
#include "ppx/internal/worldref.hpp"

namespace ppx
{
class world2D;
class behaviour2D : kit::non_copyable,
                    public kit::identifiable<std::string>,
                    public kit::toggleable,
                    public kit::yaml::codecable,
                    public worldref2D
{
  public:
    behaviour2D(world2D &world, const std::string &name);
    virtual ~behaviour2D() = default;

    void add(body2D *body);
    bool contains(const body2D *body) const;

    bool remove(std::size_t index);
    bool remove(body2D *body);

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

    const body2D &operator[](std::size_t index) const;
    body2D &operator[](std::size_t index);

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
    std::vector<body2D *> m_bodies;

  private:
    void apply_force_to_bodies();

    friend class behaviour_manager2D;
};

} // namespace ppx
