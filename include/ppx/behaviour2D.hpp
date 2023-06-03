#ifndef BEHAVIOUR2D_HPP
#define BEHAVIOUR2D_HPP
#include "ppx/core.hpp"

#include "ppx/entity2D_ptr.hpp"

namespace ppx
{
class behaviour2D
{
  public:
    behaviour2D(const char *name, std::size_t allocations = 50);
    virtual ~behaviour2D() = default;

    void validate();

    void include(const const_entity2D_ptr &e);
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

    const std::vector<const_entity2D_ptr> &entities() const;
    const char *name() const;

  protected:
    std::vector<const_entity2D_ptr> m_included;

#ifdef HAS_YAML_CPP
    virtual void write(YAML::Emitter &out) const;
    virtual YAML::Node encode() const;
    virtual bool decode(const YAML::Node &node);
#endif

    behaviour2D(const behaviour2D &) = delete;
    behaviour2D &operator=(const behaviour2D &) = delete;

  private:
    const char *m_name;
    const std::vector<entity2D> *m_entities = nullptr;
    friend class engine2D;
#ifdef HAS_YAML_CPP
    friend YAML::Emitter &operator<<(YAML::Emitter &, const behaviour2D &);
    friend struct YAML::convert<behaviour2D>;
#endif
};

#ifdef HAS_YAML_CPP
YAML::Emitter &operator<<(YAML::Emitter &out, const behaviour2D &bhv);
#endif
} // namespace ppx

#ifdef HAS_YAML_CPP
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