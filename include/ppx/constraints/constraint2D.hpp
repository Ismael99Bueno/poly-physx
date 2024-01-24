#pragma once

#include "ppx/entities/body2D.hpp"
#include "kit/interface/nameable.hpp"
#include "kit/serialization/yaml/codec.hpp"
#include "ppx/internal/worldref.hpp"

namespace ppx
{
class world2D;
class constraint2D : public kit::identifiable<>, public kit::nameable, public kit::yaml::codecable, public worldref2D
{
  public:
    constraint2D(world2D &world, const char *name);
    virtual ~constraint2D() = default;

    virtual float constraint_value() const = 0;
    virtual float constraint_velocity() const = 0;

    virtual bool contains(kit::uuid id) const = 0;
    bool contains(const body2D &body) const;

    virtual bool valid() const = 0;

    virtual void warmup() = 0;
    virtual void solve() = 0;

#ifdef KIT_USE_YAML_CPP
    virtual YAML::Node encode() const override;
    virtual bool decode(const YAML::Node &node) override;
#endif

  protected:
    float m_accumulated_lambda = 0.f;

  private:
};
} // namespace ppx
