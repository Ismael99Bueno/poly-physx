#pragma once

#include "ppx/world2D.hpp"
#include "ppx/joints/distance_joint2D.hpp"

#include "ppx/collision/detection/quad_tree_detection2D.hpp"
#include "ppx/collision/detection/brute_force_detection2D.hpp"
#include "ppx/collision/detection/sort_sweep_detection2D.hpp"
#include "ppx/collision/detection/grid_detection2D.hpp"

#include "ppx/collision/resolution/spring_driven_resolution2D.hpp"
#include "ppx/collision/resolution/constraint_driven_resolution2D.hpp"

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
        if (!node.IsMap() || node.size() < 2 || !bhv.world)
            return false;
        bhv.clear();

        bhv.enabled = node["Enabled"].as<bool>();
        for (const YAML::Node &n : node["Bodies"])
            bhv.add(bhv.world->bodies.ptr(n.as<std::size_t>()));
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::constraint2D>
{
    static YAML::Node encode(const ppx::constraint2D &ctr)
    {
        YAML::Node node;
        node["UUID"] = (std::uint64_t)ctr.id;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::constraint2D &ctr)
    {
        if (!node.IsMap() || node.size() == 0)
            return false;
        ctr.id = kit::uuid(node["UUID"].as<std::uint64_t>());
        return true;
    }
};

static void encode_joint(YAML::Node &node, const ppx::joint_proxy2D &joint)
{
    node["Index1"] = joint.body1()->index;
    node["Index2"] = joint.body2()->index;

    node["Anchor1"] = joint.rotated_anchor1();
    node["Anchor2"] = joint.rotated_anchor2();
}
static void decode_joint(const YAML::Node &node, ppx::joint_proxy2D &joint, ppx::world2D &world)
{
    const std::size_t index1 = node["Index1"].as<std::size_t>();
    const std::size_t index2 = node["Index2"].as<std::size_t>();

    const glm::vec2 a1 = node["Anchor1"].as<glm::vec2>();
    const glm::vec2 a2 = node["Anchor2"].as<glm::vec2>();

    joint.body1(world.bodies.ptr(index1));
    joint.body2(world.bodies.ptr(index2));

    joint.anchor1(a1);
    joint.anchor2(a2);
}

template <> struct kit::yaml::codec<ppx::distance_joint2D>
{
    static YAML::Node encode(const ppx::distance_joint2D &dj)
    {
        YAML::Node node;
        encode_joint(node, dj.joint);
        node["Length"] = dj.length;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::distance_joint2D &dj)
    {
        if (!node.IsMap() || node.size() != 5)
            return false;

        decode_joint(node, dj.joint, *dj.world);
        dj.length = node["Length"].as<float>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::spring2D>
{
    static YAML::Node encode(const ppx::spring2D &sp)
    {
        YAML::Node node;
        encode_joint(node, sp.joint);
        node["UUID"] = (std::uint64_t)sp.id;
        node["Stiffness"] = sp.stiffness;
        node["Damping"] = sp.damping;
        node["Length"] = sp.length;
        node["Non linear terms"] = sp.non_linear_terms;
        node["Non linear contribution"] = sp.non_linear_contribution;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::spring2D &sp)
    {
        if (!node.IsMap() || node.size() != 10)
            return false;

        decode_joint(node, sp.joint, *sp.world);

        sp.id = kit::uuid(node["UUID"].as<std::uint64_t>());
        sp.stiffness = node["Stiffness"].as<float>();
        sp.damping = node["Damping"].as<float>();
        sp.length = node["Length"].as<float>();
        sp.non_linear_terms = node["Non linear terms"].as<std::uint32_t>();
        sp.non_linear_contribution = node["Non linear contribution"].as<float>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::body2D>
{
    static YAML::Node encode(const ppx::body2D &body)
    {
        YAML::Node node;
        node["UUID"] = (std::uint64_t)body.id;
        node["Index"] = body.index;
        node["Shape"] = body.shape();
        node["Velocity"] = body.velocity;
        node["Angular velocity"] = body.angular_velocity;
        node["Mass"] = body.real_mass();
        node["Charge"] = body.charge;
        node["Kinematic"] = body.kinematic;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::body2D &body)
    {
        if (!node.IsMap() || node.size() != 8)
            return false;

        body.id = kit::uuid(node["UUID"].as<std::uint64_t>());
        body.index = node["Index"].as<std::size_t>();
        if (node["Shape"]["Radius"])
            body.shape(node["Shape"].as<geo::circle>());
        else
            body.shape(node["Shape"].as<geo::polygon>());
        body.velocity = node["Velocity"].as<glm::vec2>();
        body.angular_velocity = node["Angular velocity"].as<float>();
        body.mass(node["Mass"].as<float>());
        body.charge = node["Charge"].as<float>();
        body.kinematic = node["Kinematic"].as<bool>();

        return true;
    }
};

template <> struct kit::yaml::codec<ppx::body_manager2D>
{
    static YAML::Node encode(const ppx::body_manager2D &bm)
    {
        YAML::Node node;
        for (const ppx::body2D &body : bm)
            node["Bodies"].push_back(body);
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::body_manager2D &bm)
    {
        bm.clear();
        if (node["Bodies"])
            for (const YAML::Node &n : node["Bodies"])
                bm.add(n.as<ppx::body2D>());
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

template <> struct kit::yaml::codec<ppx::spring_manager2D>
{
    static YAML::Node encode(const ppx::spring_manager2D &sm)
    {
        YAML::Node node;
        for (const ppx::spring2D &sp : sm)
            node["Springs"].push_back(sp);
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::spring_manager2D &sm)
    {
        sm.clear();
        if (node["Springs"])
            for (const YAML::Node &n : node["Springs"])
            {
                ppx::spring2D sp;
                sp.world = &sm.world;
                n.as<ppx::spring2D>(sp);
                sm.add(sp);
            }
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::collision_manager2D>
{
    static YAML::Node encode(const ppx::collision_manager2D &cm)
    {
        YAML::Node node;
        YAML::Node ndet = node["Detection"];
        YAML::Node nqt = ndet["Quad tree"];
        ndet["EPA Threshold"] = cm.detection()->epa_threshold;
        nqt["Max bodies"] = ppx::quad_tree::max_bodies;
        nqt["Max depth"] = ppx::quad_tree::max_depth;
        nqt["Min size"] = ppx::quad_tree::min_size;

        ndet["Multithreading"] = cm.detection()->multithreaded;
        if (auto coldet = cm.detection<ppx::quad_tree_detection2D>())
        {
            ndet["Method"] = 0;
            ndet["Force square"] = coldet->force_square_shape;
        }
        else if (cm.detection<ppx::brute_force_detection2D>())
            ndet["Method"] = 1;
        else if (cm.detection<ppx::sort_sweep_detection2D>())
            ndet["Method"] = 2;
        else if (auto coldet = cm.detection<ppx::grid_detection2D>())
        {
            ndet["Method"] = 3;
            ndet["Cell size"] = coldet->cell_size;
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
        if (auto colres = cm.resolution<ppx::spring_driven_resolution2D>())
        {
            nres["Method"] = 0;
            nres["Rigidity"] = colres->rigidity;
            nres["Normal damping"] = colres->normal_damping;
            nres["Tangent damping"] = colres->tangent_damping;
        }
        else if (auto colres = cm.resolution<ppx::constraint_driven_resolution2D>())
        {
            nres["Method"] = 1;
            nres["Friction"] = colres->friction;
            nres["Restitution"] = colres->restitution;
        }
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::collision_manager2D &cm)
    {
        if (!node.IsMap() || node.size() != 2)
            return false;

        const YAML::Node ndet = node["Detection"];
        const YAML::Node nqt = ndet["Quad tree"];
        cm.detection()->epa_threshold = ndet["EPA Threshold"].as<float>();
        ppx::quad_tree::max_bodies = nqt["Max bodies"].as<std::size_t>();
        ppx::quad_tree::max_depth = nqt["Max depth"].as<std::uint32_t>();
        ppx::quad_tree::min_size = nqt["Min size"].as<float>();

        cm.detection()->multithreaded = ndet["Multithreading"].as<bool>();
        if (ndet["Method"])
        {
            const int method = ndet["Method"].as<int>();
            if (method == 0)
                cm.set_detection<ppx::quad_tree_detection2D>()->force_square_shape = ndet["Force square"].as<bool>();
            else if (method == 1)
                cm.set_detection<ppx::brute_force_detection2D>();
            else if (method == 2)
                cm.set_detection<ppx::sort_sweep_detection2D>();
            else if (method == 3)
                cm.set_detection<ppx::grid_detection2D>(ndet["Cell size"].as<float>());
        }

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
                cm.set_resolution<ppx::spring_driven_resolution2D>(nres["Rigidity"].as<float>(),
                                                                   nres["Normal damping"].as<float>(),
                                                                   nres["Tangent damping"].as<float>());
            else if (method == 1)
                cm.set_resolution<ppx::constraint_driven_resolution2D>(nres["Restitution"].as<float>(),
                                                                       nres["Friction"].as<float>());
        }
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::constraint_manager2D>
{
    static YAML::Node encode(const ppx::constraint_manager2D &cm)
    {
        YAML::Node node;
        node["Iterations"] = cm.iterations;
        node["Warmup"] = cm.warmup;
        node["Position corrections"] = cm.position_corrections;
        for (const auto &ctr : cm)
        {
            YAML::Node child;
            child[ctr->name] = *ctr;
            node["Constraints"].push_back(child);
        }
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::constraint_manager2D &cm)
    {
        if (!node.IsMap() || node.size() < 3)
            return false;

        cm.iterations = node["Iterations"].as<std::uint32_t>();
        cm.warmup = node["Warmup"].as<bool>();
        cm.position_corrections = node["Position corrections"].as<bool>();

        if (node["Constraints"])
            for (const YAML::Node &n : node["Constraints"])
                if (n["Distance"])
                {
                    ppx::distance_joint2D dj;
                    dj.world = &cm.world;
                    n["Distance"].as<ppx::distance_joint2D>(dj);
                    cm.add<ppx::distance_joint2D>(dj);
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
        node["Spring manager"] = world.springs;
        node["Collision manager"] = world.collisions;
        node["Constraint manager"] = world.constraints;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::world2D &world)
    {
        if (!node.IsMap() || node.size() != 7)
            return false;

        node["Integrator"].as<rk::integrator<float>>(world.integrator);
        world.integrator.state.clear();
        world.semi_implicit_integration = node["Semi-implicit integration"].as<bool>();
        node["Body manager"].as<ppx::body_manager2D>(world.bodies);
        node["Behaviour manager"].as<ppx::behaviour_manager2D>(world.behaviours);
        node["Spring manager"].as<ppx::spring_manager2D>(world.springs);
        node["Collision manager"].as<ppx::collision_manager2D>(world.collisions);
        node["Constraint manager"].as<ppx::constraint_manager2D>(world.constraints);

        return true;
    }
};