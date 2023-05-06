#ifndef SPRING2D_HPP
#define SPRING2D_HPP

#include "ppx/entity2D_ptr.hpp"
#include <utility>

namespace ppx
{
    class spring2D
    {
    public:
        spring2D(const entity2D_ptr &e1,
                 const entity2D_ptr &e2,
                 float stiffness = 1.f,
                 float dampening = 0.f,
                 float length = 0.f);
        spring2D(const entity2D_ptr &e1,
                 const entity2D_ptr &e2,
                 const glm::vec2 &anchor1,
                 const glm::vec2 &anchor2,
                 float stiffness = 1.f,
                 float dampening = 0.f,
                 float length = 0.f);

        std::tuple<glm::vec2, float, float> force() const;

        void bind(const entity2D_ptr &e1, const entity2D_ptr &e2);
        bool try_validate();

        float stiffness() const;
        float dampening() const;
        float length() const;

        void stiffness(float stiffness);
        void dampening(float dampening);
        void length(float length);

        float kinetic_energy() const;
        float potential_energy() const;
        float energy() const;

        const entity2D_ptr &e1() const;
        const entity2D_ptr &e2() const;

        glm::vec2 anchor1() const;
        glm::vec2 anchor2() const;

        void anchor1(const glm::vec2 &anchor1);
        void anchor2(const glm::vec2 &anchor2);

        bool has_anchors() const;

    private:
        entity2D_ptr m_e1 = nullptr, m_e2 = nullptr;
        glm::vec2 m_anchor1{0.f}, m_anchor2{0.f};
        float m_stiffness, m_dampening,
            m_angle1, m_angle2, m_length;
        bool m_has_anchors;

        std::tuple<glm::vec2, float, float> without_anchors_force() const;
        std::tuple<glm::vec2, float, float> with_anchors_force() const;
    };

#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const spring2D &sp);
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    template <>
    struct convert<ppx::spring2D>
    {
        static Node encode(const ppx::spring2D &sp);
        static bool decode(const Node &node, ppx::spring2D &sp);
    };
}
#endif
#endif