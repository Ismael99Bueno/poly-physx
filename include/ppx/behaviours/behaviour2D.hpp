#pragma once

#include "ppx/body/body2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/serialization/yaml/codec.hpp"
#include "ppx/manager2D.hpp"

namespace ppx
{
class world2D;
class behaviour2D : public manager2D<body2D>,
                    public kit::identifiable<std::string>,
                    public kit::toggleable,
                    public kit::yaml::codecable
{
  public:
    behaviour2D(world2D &world, const std::string &name);
    virtual ~behaviour2D() = default;

    virtual bool add(body2D *body);

    using manager2D<body2D>::remove;
    virtual bool remove(std::size_t index) override;

    KIT_TOGGLEABLE_FINAL_DEFAULT_SETTER()

    virtual glm::vec3 force(const state2D &state) const = 0;

    float kinetic_energy() const;
    virtual float potential_energy() const = 0;
    virtual float potential_energy(const state2D &state) const = 0;

    float energy(const body2D &body) const;
    float energy() const;

#ifdef KIT_USE_YAML_CPP
    virtual YAML::Node encode() const override;
    virtual bool decode(const YAML::Node &node) override;
#endif

  private:
    void load_forces(std::vector<state2D> &states) const;

    friend class behaviour_manager2D;
};

} // namespace ppx
