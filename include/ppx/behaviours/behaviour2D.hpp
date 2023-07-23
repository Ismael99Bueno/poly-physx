#ifndef PPX_BEHAVIOUR2D_HPP
#define PPX_BEHAVIOUR2D_HPP

#include "ppx/body2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/interface/nameable.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/interface/serialization.hpp"

namespace ppx
{
class engine2D;
class behaviour2D : kit::non_copyable,
                    public kit::identifiable<std::string>,
                    public kit::toggleable,
                    public kit::serializable
{
  public:
    behaviour2D(const std::string &name, std::size_t allocations = 50);
    virtual ~behaviour2D() = default;

    void validate();

    void include(const body2D::const_ptr &bd);
    void exclude(const body2D &bd);
    bool contains(const body2D &bd) const;

    virtual std::pair<glm::vec2, float> force(const body2D &bd) const = 0;

    float kinetic_energy() const;
    virtual float potential_energy() const = 0;
    virtual float potential_energy(const body2D &bd) const = 0;

    float energy(const body2D &bd) const;
    float energy() const;

    void clear();
    std::size_t size() const;

    const std::vector<body2D::const_ptr> &entities() const;

#ifdef KIT_USE_YAML_CPP
    virtual YAML::Node encode() const;
    virtual bool decode(const YAML::Node &node);
#endif

  protected:
    std::vector<body2D::const_ptr> m_included;

  private:
    const engine2D *m_parent = nullptr;
    friend class engine2D;
};

} // namespace ppx

#endif