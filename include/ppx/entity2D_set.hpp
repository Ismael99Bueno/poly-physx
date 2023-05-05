#ifndef ENTITY2D_SET_HPP
#define ENTITY2D_SET_HPP

#include "ppx/entity2D_ptr.hpp"

namespace ppx
{
    class entity2D_set
    {
    public:
        entity2D_set(const char *name, std::size_t allocations = 50);

        void validate();

        void include(const const_entity2D_ptr &e);
        void exclude(const entity2D &e);
        bool contains(const entity2D &e) const;
        float kinetic_energy() const;
        void clear();
        std::size_t size() const;

        const std::vector<const_entity2D_ptr> &entities() const;
        const char *name() const;

#ifdef HAS_YAML_CPP
        virtual void write(YAML::Emitter &out) const
        {
        }
        virtual YAML::Node encode() const { return YAML::Node(); }
        virtual bool decode(const YAML::Node &node) { return true; }
#endif

    protected:
        std::vector<const_entity2D_ptr> m_entities;

        entity2D_set(const entity2D_set &) = delete;
        entity2D_set &operator=(const entity2D_set &) = delete;

    private:
        const char *m_name;
    };

#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const entity2D_set &set);
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    template <>
    struct convert<ppx::entity2D_set>
    {
        static Node encode(const ppx::entity2D_set &set);
        static bool decode(const Node &node, ppx::entity2D_set &set);
    };
}
#endif

#endif