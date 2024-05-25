#pragma once

#include "ppx/world2D.hpp"
#include "ppx/joints/rotor_joint2D.hpp"
#include "ppx/joints/motor_joint2D.hpp"
#include "ppx/joints/distance_joint2D.hpp"
#include "ppx/joints/revolute_joint2D.hpp"
#include "ppx/joints/weld_joint2D.hpp"
#include "ppx/joints/ball_joint2D.hpp"
#include "ppx/joints/spring_joint2D.hpp"
#include "ppx/joints/prismatic_joint2D.hpp"

#include "ppx/collision/detection/quad_tree_detection2D.hpp"
#include "ppx/collision/detection/brute_force_detection2D.hpp"
#include "ppx/collision/detection/sort_sweep_detection2D.hpp"
#include "ppx/collision/detection/narrow/gjk_epa_detection2D.hpp"

#include "ppx/collision/contacts/spring_contact2D.hpp"
#include "ppx/collision/contacts/nonpen_contact2D.hpp"

#include "ppx/collision/manifold/clipping_algorithm_manifold2D.hpp"
#include "ppx/collision/manifold/mtv_support_manifold2D.hpp"

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

template <> struct kit::yaml::codec<ppx::specs::joint2D>
{
    static YAML::Node encode(const ppx::specs::joint2D &joint)
    {
        YAML::Node node;
        if (joint.bindex1 != SIZE_MAX)
            node["Index1"] = joint.bindex1;
        else
            node["Body1"] = joint.bspecs1;
        if (joint.bindex2 != SIZE_MAX)
            node["Index2"] = joint.bindex2;
        else
            node["Body2"] = joint.bspecs2;
        node["Bodies collide"] = joint.bodies_collide;

        return node;
    }
    static bool decode(const YAML::Node &node, ppx::specs::joint2D &joint)
    {
        if (!node.IsMap() || node.size() != 3)
            return false;

        if (node["Index1"])
            joint.bindex1 = node["Index1"].as<std::size_t>();
        else
            joint.bspecs1 = node["Body1"].as<ppx::body2D::specs>();

        if (node["Index2"])
            joint.bindex2 = node["Index2"].as<std::size_t>();
        else
            joint.bspecs2 = node["Body2"].as<ppx::body2D::specs>();
        joint.bodies_collide = node["Bodies collide"].as<bool>();

        return true;
    }
};

template <> struct kit::yaml::codec<ppx::rotor_joint2D::specs>
{
    static YAML::Node encode(const ppx::rotor_joint2D::specs &rj)
    {
        YAML::Node node;
        node["Joint"] = kit::yaml::codec<ppx::specs::joint2D>::encode(rj);
        node["Torque"] = rj.props.torque;
        node["Correction factor"] = rj.props.correction_factor;
        node["Target speed"] = rj.props.target_speed;
        node["Min angle"] = rj.props.min_angle;
        node["Max angle"] = rj.props.max_angle;
        node["Spin indefinitely"] = rj.props.spin_indefinitely;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::rotor_joint2D::specs &rj)
    {
        if (!node.IsMap() || node.size() != 7)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], rj))
            return false;
        rj.props.torque = node["Torque"].as<float>();
        rj.props.correction_factor = node["Correction factor"].as<float>();
        rj.props.target_speed = node["Target speed"].as<float>();
        rj.props.min_angle = node["Min angle"].as<float>();
        rj.props.max_angle = node["Max angle"].as<float>();
        rj.props.spin_indefinitely = node["Spin indefinitely"].as<bool>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::motor_joint2D::specs>
{
    static YAML::Node encode(const ppx::motor_joint2D::specs &mj)
    {
        YAML::Node node;
        node["Joint"] = kit::yaml::codec<ppx::specs::joint2D>::encode(mj);
        node["Force"] = mj.props.force;
        node["Correction factor"] = mj.props.correction_factor;
        node["Target speed"] = mj.props.target_speed;
        node["Target offset"] = mj.props.target_offset;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::motor_joint2D::specs &mj)
    {
        if (!node.IsMap() || node.size() != 5)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], mj))
            return false;
        mj.props.force = node["Force"].as<float>();
        mj.props.correction_factor = node["Correction factor"].as<float>();
        mj.props.target_speed = node["Target speed"].as<float>();
        mj.props.target_offset = node["Target offset"].as<glm::vec2>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::distance_joint2D::specs>
{
    static YAML::Node encode(const ppx::distance_joint2D::specs &dj)
    {
        YAML::Node node;
        node["Joint"] = kit::yaml::codec<ppx::specs::joint2D>::encode(dj);
        node["Anchor1"] = dj.ganchor1;
        node["Anchor2"] = dj.ganchor2;
        node["Min distance"] = dj.props.min_distance;
        node["Max distance"] = dj.props.max_distance;
        node["Deduce distance"] = dj.deduce_distance;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::distance_joint2D::specs &dj)
    {
        if (!node.IsMap() || node.size() != 6)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], dj))
            return false;
        dj.ganchor1 = node["Anchor1"].as<glm::vec2>();
        dj.ganchor2 = node["Anchor2"].as<glm::vec2>();
        dj.props.min_distance = node["Min distance"].as<float>();
        dj.props.max_distance = node["Max distance"].as<float>();
        dj.deduce_distance = node["Deduce distance"].as<bool>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::ball_joint2D::specs>
{
    static YAML::Node encode(const ppx::ball_joint2D::specs &bj)
    {
        YAML::Node node;
        node["Joint"] = kit::yaml::codec<ppx::specs::joint2D>::encode(bj);
        node["Deduce angle"] = bj.deduce_angle;
        node["Min angle"] = bj.props.min_angle;
        node["Max angle"] = bj.props.max_angle;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::ball_joint2D::specs &bj)
    {
        if (!node.IsMap() || node.size() != 4)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], bj))
            return false;
        bj.deduce_angle = node["Deduce angle"].as<bool>();
        bj.props.min_angle = node["Min angle"].as<float>();
        bj.props.max_angle = node["Max angle"].as<float>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::prismatic_joint2D::specs>
{
    static YAML::Node encode(const ppx::prismatic_joint2D::specs &pj)
    {
        YAML::Node node;
        node["Joint"] = kit::yaml::codec<ppx::specs::joint2D>::encode(pj);
        node["Anchor1"] = pj.ganchor1;
        node["Anchor2"] = pj.ganchor2;
        node["Deduce axis"] = pj.deduce_axis;
        node["Axis"] = pj.props.axis;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::prismatic_joint2D::specs &pj)
    {
        if (!node.IsMap() || node.size() != 5)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], pj))
            return false;
        pj.ganchor1 = node["Anchor1"].as<glm::vec2>();
        pj.ganchor2 = node["Anchor2"].as<glm::vec2>();
        pj.deduce_axis = node["Deduce axis"].as<bool>();
        pj.props.axis = node["Axis"].as<glm::vec2>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::revolute_joint2D::specs>
{
    static YAML::Node encode(const ppx::revolute_joint2D::specs &revj)
    {
        YAML::Node node;
        node["Joint"] = kit::yaml::codec<ppx::specs::joint2D>::encode(revj);
        node["Anchor"] = revj.ganchor;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::revolute_joint2D::specs &revj)
    {
        if (!node.IsMap() || node.size() != 2)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], revj))
            return false;
        revj.ganchor = node["Anchor"].as<glm::vec2>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::weld_joint2D::specs>
{
    static YAML::Node encode(const ppx::weld_joint2D::specs &weldj)
    {
        YAML::Node node;
        node["Joint"] = kit::yaml::codec<ppx::specs::joint2D>::encode(weldj);
        node["Anchor"] = weldj.ganchor;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::weld_joint2D::specs &weldj)
    {
        if (!node.IsMap() || node.size() != 2)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], weldj))
            return false;
        weldj.ganchor = node["Anchor"].as<glm::vec2>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::spring_joint2D::specs>
{
    static YAML::Node encode(const ppx::spring_joint2D::specs &sp)
    {
        YAML::Node node;
        node["Joint"] = kit::yaml::codec<ppx::specs::joint2D>::encode(sp);
        node["Anchor1"] = sp.ganchor1;
        node["Anchor2"] = sp.ganchor2;
        node["Frequency"] = sp.props.frequency;
        node["Damping ratio"] = sp.props.damping_ratio;
        node["Min length"] = sp.props.min_length;
        node["Max length"] = sp.props.max_length;
        node["Deduce length"] = sp.deduce_length;
        node["Non linear terms"] = sp.props.non_linear_terms;
        node["Non linear contribution"] = sp.props.non_linear_contribution;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::spring_joint2D::specs &sp)
    {
        if (!node.IsMap() || node.size() != 10)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], sp))
            return false;
        sp.ganchor1 = node["Anchor1"].as<glm::vec2>();
        sp.ganchor2 = node["Anchor2"].as<glm::vec2>();
        sp.props.frequency = node["Frequency"].as<float>();
        sp.props.damping_ratio = node["Damping ratio"].as<float>();
        sp.props.min_length = node["Min length"].as<float>();
        sp.props.max_length = node["Max length"].as<float>();
        sp.deduce_length = node["Deduce length"].as<bool>();
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

template <ppx::Joint2D T> struct kit::yaml::codec<ppx::joint_manager2D<T>>
{
    static YAML::Node encode(const ppx::joint_manager2D<T> &jc)
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
    static bool decode(const YAML::Node &node, ppx::joint_manager2D<T> &jc)
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

template <ppx::IManager IM> struct kit::yaml::codec<ppx::joint_meta_manager2D<IM>>
{
    static YAML::Node encode(const ppx::joint_meta_manager2D<IM> &mm)
    {
        YAML::Node node;
        for (const auto &mng : mm)
            node["Managers"][mng->id] = *mng;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::joint_meta_manager2D<IM> &mm)
    {
        if (node["Managers"])
            for (auto it = node["Managers"].begin(); it != node["Managers"].end(); ++it)
            {
                const auto manager = mm[it->first.as<std::string>()];
                KIT_ASSERT_WARN(manager,
                                "The joint manager {0} does not exist in the current simulation, so it will not be "
                                "deserialized. Consider adding it before deserialization",
                                it->first.as<std::string>())
                if (manager)
                    it->second.as<IM>(*manager);
            }
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::joint_repository2D>
{
    static YAML::Node encode(const ppx::joint_repository2D &jr)
    {
        YAML::Node node;
        node["Actuators"] = kit::yaml::codec<ppx::joint_meta_manager2D<ppx::iactuator_manager2D>>::encode(jr.actuators);
        node["Constraints"] =
            kit::yaml::codec<ppx::joint_meta_manager2D<ppx::iconstraint_manager2D>>::encode(jr.constraints);
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::joint_repository2D &jr)
    {
        if (!node.IsMap() || node.size() != 2)
            return false;
        kit::yaml::codec<ppx::joint_meta_manager2D<ppx::iactuator_manager2D>>::decode(node["Actuators"], jr.actuators);
        kit::yaml::codec<ppx::joint_meta_manager2D<ppx::iconstraint_manager2D>>::decode(node["Constraints"],
                                                                                        jr.constraints);
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
            ndet["Detection method"] = 0;
        else if (auto coldet = cm.detection<ppx::quad_tree_detection2D>())
        {
            ndet["Detection method"] = 1;
            ndet["Force square"] = coldet->force_square_shape;

            const auto &props = coldet->quad_tree().props();
            ndet["Max colliders"] = props.elements_per_quad;
            ndet["Max depth"] = props.max_depth;
            ndet["Min size"] = props.min_quad_size;
        }
        else if (cm.detection<ppx::sort_sweep_detection2D>())
            ndet["Detection method"] = 2;

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

        if (auto clip = cm.detection()->pp_manifold_algorithm<ppx::clipping_algorithm_manifold2D>())
            ndet["P-P Algorithm"] = 0;
        else if (cm.detection()->pp_manifold_algorithm<ppx::mtv_support_manifold2D>())
            ndet["P-P Algorithm"] = 1;

        YAML::Node nsolv = node["Contacts"];
        nsolv["Contact lifetime"] = cm.contacts()->contact_lifetime;
        if (auto colsolv = cm.contacts<ppx::contact_solver2D<ppx::nonpen_contact2D>>())
            nsolv["Solver method"] = 0;
        else if (auto colsolv = cm.contacts<ppx::contact_solver2D<ppx::spring_contact2D>>())
            nsolv["Solver method"] = 1;

        nsolv["Rigidity"] = ppx::spring_contact2D::rigidity;
        nsolv["Max normal damping"] = ppx::spring_contact2D::max_normal_damping;
        nsolv["Max tangent damping"] = ppx::spring_contact2D::max_tangent_damping;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::collision_manager2D &cm)
    {
        if (!node.IsMap() || node.size() != 2)
            return false;

        const YAML::Node ndet = node["Detection"];

        cm.detection()->multithreaded = ndet["Multithreading"].as<bool>();
        if (ndet["Detection method"])
        {
            const int method = ndet["Detection method"].as<int>();
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

        if (ndet["C-P Narrow"]) // bc there is only one for now
            cm.detection()->set_cp_narrow_detection<ppx::gjk_epa_detection2D>(ndet["C-P EPA Threshold"].as<float>());
        if (ndet["P-P Narrow"])
            cm.detection()->set_pp_narrow_detection<ppx::gjk_epa_detection2D>(ndet["P-P EPA Threshold"].as<float>());

        if (ndet["P-P Algorithm"])
        {
            const int alg = ndet["P-P Algorithm"].as<int>();
            if (alg == 0)
                cm.detection()->set_pp_manifold_algorithm<ppx::clipping_algorithm_manifold2D>();
            else if (alg == 1)
                cm.detection()->set_pp_manifold_algorithm<ppx::mtv_support_manifold2D>();
        }

        const YAML::Node nsolv = node["Contacts"];
        if (nsolv["Solver method"])
        {
            const int method = nsolv["Solver method"].as<int>();
            if (method == 0)
                cm.set_contact_solver<ppx::contact_solver2D<ppx::nonpen_contact2D>>();
            else if (method == 1)
                cm.set_contact_solver<ppx::contact_solver2D<ppx::spring_contact2D>>();
        }
        cm.contacts()->contact_lifetime = nsolv["Contact lifetime"].as<float>();

        ppx::spring_contact2D::rigidity = nsolv["Rigidity"].as<float>();
        ppx::spring_contact2D::max_normal_damping = nsolv["Max normal damping"].as<float>();
        ppx::spring_contact2D::max_tangent_damping = nsolv["Max tangent damping"].as<float>();
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
        ctrs["Velocity iterations"] = world.constraints.velocity_iterations;
        ctrs["Position iterations"] = world.constraints.position_iterations;
        ctrs["Warmup"] = world.constraints.warmup;
        ctrs["Baumgarte correction"] = world.constraints.baumgarte_correction;
        ctrs["Baumgarte coef"] = world.constraints.baumgarte_coef;
        ctrs["Baumgarte threshold"] = world.constraints.baumgarte_threshold;
        ctrs["Slop"] = world.constraints.slop;
        ctrs["Max position correction"] = world.constraints.max_position_correction;
        ctrs["Position resolution speed"] = world.constraints.position_resolution_speed;

        YAML::Node islands = node["Island settings"];
        islands["Enabled"] = world.islands.enabled();
        islands["Sleep energy threshold"] = world.islands.sleep_energy_threshold;
        islands["Sleep time threshold"] = world.islands.sleep_time_threshold;

        return node;
    }
    static bool decode(const YAML::Node &node, ppx::world2D &world)
    {
        if (!node.IsMap() || node.size() != 8)
            return false;

        world.semi_implicit_integration = node["Semi-implicit integration"].as<bool>();
        node["Body manager"].as<ppx::body_manager2D>(world.bodies);
        node["Integrator"].as<rk::integrator<float>>(world.integrator);
        node["Behaviour manager"].as<ppx::behaviour_manager2D>(world.behaviours);
        node["Collision manager"].as<ppx::collision_manager2D>(world.collisions);
        node["Joints repository"].as<ppx::joint_repository2D>(world.joints);

        const YAML::Node ctrs = node["Constraint settings"];
        world.constraints.velocity_iterations = ctrs["Velocity iterations"].as<std::uint32_t>();
        world.constraints.position_iterations = ctrs["Position iterations"].as<std::uint32_t>();
        world.constraints.warmup = ctrs["Warmup"].as<bool>();
        world.constraints.baumgarte_correction = ctrs["Baumgarte correction"].as<bool>();
        world.constraints.baumgarte_coef = ctrs["Baumgarte coef"].as<float>();
        world.constraints.baumgarte_threshold = ctrs["Baumgarte threshold"].as<float>();
        world.constraints.slop = ctrs["Slop"].as<float>();
        world.constraints.max_position_correction = ctrs["Max position correction"].as<float>();
        world.constraints.position_resolution_speed = ctrs["Position resolution speed"].as<float>();

        const YAML::Node islands = node["Island settings"];
        world.islands.enabled(islands["Enabled"].as<bool>());
        world.islands.sleep_energy_threshold = islands["Sleep energy threshold"].as<float>();
        world.islands.sleep_time_threshold = islands["Sleep time threshold"].as<float>();

        return true;
    }
};