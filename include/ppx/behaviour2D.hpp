#ifndef BEHAVIOUR2D_HPP
#define BEHAVIOUR2D_HPP

#include "ppx/entity2D_ptr.hpp"

namespace ppx
{
    class behaviour2D
    {
    public:
        behaviour2D(const char *name, std::size_t allocations = 50);

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

        behaviour2D(const behaviour2D &) = delete;
        behaviour2D &operator=(const behaviour2D &) = delete;

    private:
        const char *m_name;
    };

#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const behaviour2D &set);
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    template <>
    struct convert<ppx::behaviour2D>
    {
        static Node encode(const ppx::behaviour2D &set);
        static bool decode(const Node &node, ppx::behaviour2D &set);
    };
}
#endif

#endif