#ifndef PPX_BEHAVIOUR2D_HPP
#define PPX_BEHAVIOUR2D_HPP

#include "ppx/entity2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/interface/nameable.hpp"
#include "kit/interface/toggleable.hpp"

namespace ppx
{
class behaviour2D : kit::non_copyable, public kit::identifiable, public kit::nameable, public kit::toggleable
{
  public:
    behaviour2D(const char *name, std::size_t allocations = 50);
    virtual ~behaviour2D() = default;

    void validate();

    void include(const entity2D::const_ptr &e);
    void exclude(const entity2D &e);
    bool contains(const entity2D &e) const;

    virtual std::pair<glm::vec2, float> force(const entity2D &e) const = 0;

    float kinetic_energy() const;
    virtual float potential_energy() const = 0;
    virtual float potential_energy(const entity2D &e) const = 0;

    float energy(const entity2D &e) const;
    float energy() const;

    void clear();
    std::size_t size() const;

    const std::vector<entity2D::const_ptr> &entities() const;

  protected:
    std::vector<entity2D::const_ptr> m_included;

#ifdef KIT_USE_YAML_CPP
    virtual void write(YAML::Emitter &out) const;
    virtual YAML::Node encode() const;
    virtual bool decode(const YAML::Node &node);
#endif

  private:
    const kit::track_vector<entity2D> *m_entities = nullptr;

    friend class engine2D;
#ifdef KIT_USE_YAML_CPP
    friend YAML::Emitter &operator<<(YAML::Emitter &, const behaviour2D &);
    friend struct YAML::convert<behaviour2D>;
#endif
};

#ifdef KIT_USE_YAML_CPP
YAML::Emitter &operator<<(YAML::Emitter &out, const behaviour2D &bhv);
#endif
} // namespace ppx

#ifdef KIT_USE_YAML_CPP
namespace YAML
{
template <> struct convert<ppx::behaviour2D>
{
    static Node encode(const ppx::behaviour2D &bhv);
    static bool decode(const Node &node, ppx::behaviour2D &bhv);
};
} // namespace YAML
#endif

#endif