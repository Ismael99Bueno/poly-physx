#ifndef PPX_CONSTRAINT2D_HPP
#define PPX_CONSTRAINT2D_HPP

#include "ppx/body2D.hpp"
#include "kit/interface/nameable.hpp"

namespace ppx
{
class world2D;
class constraint2D : public kit::identifiable<>, public kit::serializable, public kit::nameable
{
  public:
    constraint2D(const char *name);
    virtual ~constraint2D() = default;

#ifdef KIT_USE_YAML_CPP
    virtual YAML::Node encode() const override;
    virtual bool decode(const YAML::Node &node) override;
#endif

    virtual float constraint_value() const = 0;
    virtual float constraint_derivative() const = 0;

    virtual bool contains(kit::uuid id) const = 0;
    bool contais(const body2D &body) const;

  protected:
    world2D *m_world = nullptr;

  private:
    virtual bool valid() const = 0;

    virtual void warmup() = 0;
    virtual void solve() = 0;

    friend class constraint_manager2D;
};
} // namespace ppx

#endif