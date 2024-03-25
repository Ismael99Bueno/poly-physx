#pragma once

#include "ppx/world2D.hpp"
#include "ppx/joints/distance_joint2D.hpp"
#include "ppx/joints/spring2D.hpp"

#include "ppx/collision/detection/quad_tree_detection2D.hpp"
#include "ppx/collision/detection/brute_force_detection2D.hpp"
#include "ppx/collision/detection/sort_sweep_detection2D.hpp"

#include "ppx/collision/resolution/sequential_impulses_resolution2D.hpp"
#include "ppx/collision/resolution/spring_driven_resolution2D.hpp"

#include "ppx/collision/detection/narrow/gjk_epa_detection2D.hpp"

#include "ppx/collision/manifold/clipping_algorithm_manifold2D.hpp"
#include "ppx/collision/manifold/mtv_support_manifold2D.hpp"
#include "ppx/collision/manifold/radius_distance_manifold2D.hpp"

#include "rk/serialization/serialization.hpp"

#include "kit/serialization/yaml/codec.hpp"
#include "kit/serialization/yaml/glm.hpp"

template <> struct kit::yaml::codec<ppx::behaviour2D>
{
    static YAML::Node encode(const ppx::behaviour2D &bhv)
    {
        YAML::Node node;
        node["Enabled"] = bhv.enabled;

        for (const auto &body : bhv)
            node["Bodies"].push_back(body->index);
        node["Bodies"].SetStyle(YAML::EmitterStyle::Flow);
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::behaviour2D &bhv)
    {
        if (!node.IsMap() || node.size() < 2)
            return false;
        bhv.clear();

        bhv.enabled = node["Enabled"].as<bool>();
        for (const YAML::Node &n : node["Bodies"])
            bhv.add(bhv.world.bodies[n.as<std::size_t>()]);
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::collider2D::specs>
{
    static YAML::Node encode(const ppx::collider2D::specs &collider)
    {
        YAML::Node node;
        node["Position"] = collider.position;
        node["Rotation"] = collider.rotation;
        node["Density"] = collider.props.density;
        node["Charge density"] = collider.props.charge_density;
        node["Restitution"] = collider.props.restitution;
        node["Friction"] = collider.props.friction;
        switch (collider.props.shape)
        {
        case ppx::collider2D::stype::CIRCLE:
            node["Radius"] = collider.props.radius;
            break;
        case ppx::collider2D::stype::POLYGON:
            for (const glm::vec2 &v : collider.props.vertices)
                node["Vertices"].push_back(v);
        }

        return node;
    }
    static bool decode(const YAML::Node &node, ppx::collider2D::specs &collider)
    {
        if (!node.IsMap() || node.size() != 7)
            return false;

        collider.position = node["Position"].as<glm::vec2>();
        collider.rotation = node["Rotation"].as<float>();
        collider.props.density = node["Density"].as<float>();
        collider.props.charge_density = node["Charge density"].as<float>();
        collider.props.restitution = node["Restitution"].as<float>();
        collider.props.friction = node["Friction"].as<float>();
        if (node["Radius"])
        {
            collider.props.radius = node["Radius"].as<float>();
            collider.props.shape = ppx::collider2D::stype::CIRCLE;
        }
        else if (node["Vertices"])
        {
            collider.props.vertices.clear();
            for (const YAML::Node &n : node["Vertices"])
                collider.props.vertices.push_back(n.as<glm::vec2>());
            collider.props.shape = ppx::collider2D::stype::POLYGON;
        }
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::body2D::specs>
{
    static YAML::Node encode(const ppx::body2D::specs &body)
    {
        YAML::Node node;
        node["Position"] = body.position;
        node["Velocity"] = body.velocity;
        node["Rotation"] = body.rotation;
        node["Angular velocity"] = body.angular_velocity;
        node["Mass"] = body.props.mass;
        node["Charge"] = body.props.charge;
        node["Type"] = (int)body.props.type;
        for (const ppx::collider2D::specs &collider : body.props.colliders)
            node["Colliders"].push_back(collider);
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::body2D::specs &body)
    {
        if (!node.IsMap() || node.size() < 7)
            return false;

        body.position = node["Position"].as<glm::vec2>();
        body.velocity = node["Velocity"].as<glm::vec2>();
        body.rotation = node["Rotation"].as<float>();
        body.angular_velocity = node["Angular velocity"].as<float>();
        body.props.mass = node["Mass"].as<float>();
        body.props.charge = node["Charge"].as<float>();
        body.props.type = (ppx::body2D::btype)node["Type"].as<int>();
        if (node["Colliders"])
            for (const YAML::Node &n : node["Colliders"])
                body.props.colliders.push_back(n.as<ppx::collider2D::specs>());

        return true;
    }
};

template <> struct kit::yaml::codec<ppx::distance_joint2D::specs>
{
    static YAML::Node encode(const ppx::distance_joint2D::specs &dj)
    {
        YAML::Node node;
        node["Index1"] = dj.bindex1;
        node["Index2"] = dj.bindex2;
        node["Anchor1"] = dj.ganchor1;
        node["Anchor2"] = dj.ganchor2;
        node["Min distance"] = dj.min_distance;
        node["Max distance"] = dj.max_distance;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::distance_joint2D::specs &dj)
    {
        if (!node.IsMap() || node.size() != 6)
            return false;

        dj.bindex1 = node["Index1"].as<std::size_t>();
        dj.bindex2 = node["Index2"].as<std::size_t>();
        dj.ganchor1 = node["Anchor1"].as<glm::vec2>();
        dj.ganchor2 = node["Anchor2"].as<glm::vec2>();
        dj.min_distance = node["Min distance"].as<float>();
        dj.max_distance = node["Max distance"].as<float>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::spring2D::specs>
{
    static YAML::Node encode(const ppx::spring2D::specs &sp)
    {
        YAML::Node node;
        node["Index1"] = sp.bindex1;
        node["Index2"] = sp.bindex2;
        node["Anchor1"] = sp.ganchor1;
        node["Anchor2"] = sp.ganchor2;
        node["Frequency"] = sp.props.frequency;
        node["Damping ratio"] = sp.props.damping_ratio;
        node["Length"] = sp.props.length;
        node["Non linear terms"] = sp.props.non_linear_terms;
        node["Non linear contribution"] = sp.props.non_linear_contribution;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::spring2D::specs &sp)
    {
        if (!node.IsMap() || node.size() != 9)
            return false;

        sp.bindex1 = node["Index1"].as<std::size_t>();
        sp.bindex2 = node["Index2"].as<std::size_t>();
        sp.ganchor1 = node["Anchor1"].as<glm::vec2>();
        sp.ganchor2 = node["Anchor2"].as<glm::vec2>();
        sp.props.frequency = node["Frequency"].as<float>();
        sp.props.damping_ratio = node["Damping ratio"].as<float>();
        sp.props.length = node["Length"].as<float>();
        sp.props.non_linear_terms = node["Non linear terms"].as<std::uint32_t>();
        sp.props.non_linear_contribution = node["Non linear contribution"].as<float>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::body_manager2D>
{
    static YAML::Node encode(const ppx::body_manager2D &bm)
    {
        YAML::Node node;
        for (const ppx::body2D *body : bm)
            node["Bodies"].push_back(ppx::body2D::specs::from_instance(*body));
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::body_manager2D &bm)
    {
        bm.clear();
        if (node["Bodies"])
            for (const YAML::Node &n : node["Bodies"])
            {
                const ppx::body2D::specs specs = n.as<ppx::body2D::specs>();
                bm.add(specs);
            }
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::behaviour_manager2D>
{
    static YAML::Node encode(const ppx::behaviour_manager2D &bm)
    {
        YAML::Node node;
        for (const auto &bhv : bm)
            node["Behaviours"][bhv->id] = *bhv;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::behaviour_manager2D &bm)
    {
        if (node["Behaviours"])
            for (auto it = node["Behaviours"].begin(); it != node["Behaviours"].end(); ++it)
            {
                const auto bhv = bm[it->first.as<std::string>()];
                if (bhv)
                    it->second.as<ppx::behaviour2D>(*bhv);
            }
        return true;
    }
};

template <ppx::Joint T> struct kit::yaml::codec<ppx::joint_container2D<T>>
{
    static YAML::Node encode(const ppx::joint_container2D<T> &jc)
    {
        YAML::Node node;
        using specs = typename T::specs;
        if constexpr (kit::yaml::Encodeable<specs>)
        {
            for (const T *joint : jc)
                node["Joints"].push_back(specs::from_instance(*joint));
            return node;
        }
        else
        {
            KIT_WARN("The joint type {0} specifications are not encodeable, so the joint instances will not be "
                     "serialized. Consider "
                     "implementing encoding "
                     "functionality for the joint type",
                     typeid(T).name())
            return node;
        }
    }
    static bool decode(const YAML::Node &node, ppx::joint_container2D<T> &jc)
    {
        using specs = typename T::specs;
        if constexpr (kit::yaml::Decodeable<specs>)
        {
            if (node["Joints"])
                for (const YAML::Node &n : node["Joints"])
                    jc.add(n.as<specs>());
            return true;
        }
        else
        {
            KIT_WARN("The joint type {0} specifications are not decodeable, so the joint instances will not be "
                     "serialized. Consider "
                     "implementing encoding "
                     "functionality for the joint type",
                     typeid(T).name())
            return true;
        }
    }
};

template <ppx::Solver S> struct kit::yaml::codec<ppx::meta_manager2D<S>>
{
    static YAML::Node encode(const ppx::meta_manager2D<S> &mm)
    {
        YAML::Node node;
        for (const auto &mng : mm)
            node["Managers"][mng->id] = *mng;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::meta_manager2D<S> &mm)
    {
        if (node["Managers"])
            for (auto it = node["Managers"].begin(); it != node["Managers"].end(); ++it)
            {
                const auto solver = mm[it->first.as<std::string>()];
                KIT_ASSERT_WARN(solver,
                                "The joint manager {0} does not exist in the current simulation, so it will not be "
                                "deserialized. Consider adding it before deserialization",
                                it->first.as<std::string>())
                if (solver)
                    it->second.as<S>(*solver);
            }
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::joint_repository2D>
{
    static YAML::Node encode(const ppx::joint_repository2D &jr)
    {
        YAML::Node node;
        node["Non constraint based"] =
            kit::yaml::codec<ppx::meta_manager2D<ppx::joint_solver2D>>::encode(jr.non_constraint_based);
        node["Constraint based"] =
            kit::yaml::codec<ppx::meta_manager2D<ppx::constraint_solver2D>>::encode(jr.constraint_based);
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::joint_repository2D &jr)
    {
        if (!node.IsMap() || node.size() != 2)
            return false;
        kit::yaml::codec<ppx::meta_manager2D<ppx::joint_solver2D>>::decode(node["Non constraint based"],
                                                                           jr.non_constraint_based);
        kit::yaml::codec<ppx::meta_manager2D<ppx::constraint_solver2D>>::decode(node["Constraint based"],
                                                                                jr.constraint_based);
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::collision_manager2D>
{
    static YAML::Node encode(const ppx::collision_manager2D &cm)
    {
        YAML::Node node;
        YAML::Node ndet = node["Detection"];

        ndet["Multithreading"] = cm.detection()->multithreaded;
        if (cm.detection<ppx::brute_force_detection2D>())
            ndet["Method"] = 0;
        else if (auto coldet = cm.detection<ppx::quad_tree_detection2D>())
        {
            ndet["Method"] = 1;
            ndet["Force square"] = coldet->force_square_shape;

            const auto &props = coldet->quad_tree().props();
            ndet["Max colliders"] = props.elements_per_quad;
            ndet["Max depth"] = props.max_depth;
            ndet["Min size"] = props.min_quad_size;
        }

        else if (cm.detection<ppx::sort_sweep_detection2D>())
            ndet["Method"] = 2;

        if (auto narrow = cm.detection()->cp_narrow_detection<ppx::gjk_epa_detection2D>())
        {
            ndet["C-P Narrow"] = 0;
            ndet["C-P EPA Threshold"] = narrow->epa_threshold;
        }
        if (auto narrow = cm.detection()->pp_narrow_detection<ppx::gjk_epa_detection2D>())
        {
            ndet["P-P Narrow"] = 0;
            ndet["P-P EPA Threshold"] = narrow->epa_threshold;
        }

        if (cm.detection()->cc_manifold_algorithm<ppx::radius_distance_manifold2D>())
            ndet["C-C Algorithm"] = 0;
        else if (cm.detection()->cc_manifold_algorithm<ppx::mtv_support_manifold2D>())
            ndet["C-C Algorithm"] = 1;

        if (cm.detection()->cp_manifold_algorithm<ppx::mtv_support_manifold2D>())
            ndet["C-P Algorithm"] = 0;

        if (cm.detection()->pp_manifold_algorithm<ppx::clipping_algorithm_manifold2D>())
            ndet["P-P Algorithm"] = 0;
        else if (cm.detection()->pp_manifold_algorithm<ppx::mtv_support_manifold2D>())
            ndet["P-P Algorithm"] = 1;

        YAML::Node nres = node["Resolution"];
        if (auto colres = cm.resolution<ppx::sequential_impulses_resolution2D>())
        {
            nres["Method"] = 0;
            nres["Slop"] = colres->slop;
        }
        else if (auto colres = cm.resolution<ppx::spring_driven_resolution2D>())
        {
            nres["Method"] = 1;
            nres["Rigidity"] = colres->rigidity;
            nres["Normal damping"] = colres->normal_damping;
            nres["Tangent damping"] = colres->tangent_damping;
        }

        return node;
    }
    static bool decode(const YAML::Node &node, ppx::collision_manager2D &cm)
    {
        if (!node.IsMap() || node.size() != 2)
            return false;

        const YAML::Node ndet = node["Detection"];

        cm.detection()->multithreaded = ndet["Multithreading"].as<bool>();
        if (ndet["Method"])
        {
            const int method = ndet["Method"].as<int>();
            if (method == 0)
                cm.set_detection<ppx::brute_force_detection2D>();
            else if (method == 1)
            {
                auto qtdet = cm.set_detection<ppx::quad_tree_detection2D>();
                qtdet->force_square_shape = ndet["Force square"].as<bool>();

                auto &props = qtdet->quad_tree().props();
                props.elements_per_quad = ndet["Max colliders"].as<std::size_t>();
                props.max_depth = ndet["Max depth"].as<std::uint32_t>();
                props.min_quad_size = ndet["Min size"].as<float>();
            }
            else if (method == 2)
                cm.set_detection<ppx::sort_sweep_detection2D>();
        }

        if (ndet["C-P Narrow"])
            cm.detection()->set_cp_narrow_detection<ppx::gjk_epa_detection2D>(ndet["C-P EPA Threshold"].as<float>());
        if (ndet["P-P Narrow"])
            cm.detection()->set_pp_narrow_detection<ppx::gjk_epa_detection2D>(ndet["P-P EPA Threshold"].as<float>());

        if (ndet["C-C Algorithm"])
        {
            const int alg = ndet["C-C Algorithm"].as<int>();
            if (alg == 0)
                cm.detection()->set_pp_manifold_algorithm<ppx::clipping_algorithm_manifold2D>();
            else if (alg == 1)
                cm.detection()->set_pp_manifold_algorithm<ppx::mtv_support_manifold2D>();
        }

        if (ndet["C-P Algorithm"])
        {
            const int alg = ndet["C-P Algorithm"].as<int>();
            if (alg == 0)
                cm.detection()->set_cp_manifold_algorithm<ppx::mtv_support_manifold2D>();
        }

        if (ndet["P-P Algorithm"])
        {
            const int alg = ndet["P-P Algorithm"].as<int>();
            if (alg == 0)
                cm.detection()->set_pp_manifold_algorithm<ppx::clipping_algorithm_manifold2D>();
            else if (alg == 1)
                cm.detection()->set_pp_manifold_algorithm<ppx::mtv_support_manifold2D>();
        }

        const YAML::Node nres = node["Resolution"];
        if (nres["Method"])
        {
            const int method = nres["Method"].as<int>();
            if (method == 0)
                cm.set_resolution<ppx::sequential_impulses_resolution2D>(nres["Slop"].as<float>());
            else if (method == 1)
                cm.set_resolution<ppx::spring_driven_resolution2D>(nres["Rigidity"].as<float>(),
                                                                   nres["Normal damping"].as<float>(),
                                                                   nres["Tangent damping"].as<float>());
        }
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::world2D>
{
    static YAML::Node encode(const ppx::world2D &world)
    {
        YAML::Node node;
        node["Integrator"] = world.integrator;
        node["Semi-implicit integration"] = world.semi_implicit_integration;
        node["Body manager"] = world.bodies;
        node["Behaviour manager"] = world.behaviours;
        node["Collision manager"] = world.collisions;
        node["Joints repository"] = world.joints;

        YAML::Node ctrs = node["Constraint settings"];
        ctrs["Iterations"] = world.constraints.iterations;
        ctrs["Warmup"] = world.constraints.warmup;
        ctrs["Baumgarte correction"] = world.constraints.baumgarte_correction;
        ctrs["Baumgarte coef"] = world.constraints.baumgarte_coef;
        ctrs["Baumgarte threshold"] = world.constraints.baumgarte_threshold;

        return node;
    }
    static bool decode(const YAML::Node &node, ppx::world2D &world)
    {
        if (!node.IsMap() || node.size() != 7)
            return false;

        world.semi_implicit_integration = node["Semi-implicit integration"].as<bool>();
        node["Body manager"].as<ppx::body_manager2D>(world.bodies);
        node["Integrator"].as<rk::integrator<float>>(world.integrator);
        node["Behaviour manager"].as<ppx::behaviour_manager2D>(world.behaviours);
        node["Collision manager"].as<ppx::collision_manager2D>(world.collisions);
        node["Joints repository"].as<ppx::joint_repository2D>(world.joints);

        const YAML::Node ctrs = node["Constraint settings"];
        world.constraints.iterations = ctrs["Iterations"].as<std::uint32_t>();
        world.constraints.warmup = ctrs["Warmup"].as<bool>();
        world.constraints.baumgarte_correction = ctrs["Baumgarte correction"].as<bool>();
        world.constraints.baumgarte_coef = ctrs["Baumgarte coef"].as<float>();
        world.constraints.baumgarte_threshold = ctrs["Baumgarte threshold"].as<float>();

        return true;
    }
};