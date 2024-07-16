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

#include "ppx/collision/broad/quad_tree_broad2D.hpp"
#include "ppx/collision/broad/brute_force_broad2D.hpp"
#include "ppx/collision/broad/sort_sweep_broad2D.hpp"
#include "ppx/collision/narrow/gjk_epa_narrow2D.hpp"
#include "ppx/collision/narrow/sat_narrow2D.hpp"

#include "ppx/collision/contacts/spring_contact2D.hpp"
#include "ppx/collision/contacts/nonpen_contact2D.hpp"

#include "rk/serialization/serialization.hpp"

#include "kit/serialization/yaml/codec.hpp"
#include "kit/serialization/yaml/glm.hpp"

template <> struct kit::yaml::codec<ppx::behaviour2D>
{
    static YAML::Node encode(const ppx::behaviour2D &bhv)
    {
        YAML::Node node;
        node["Enabled"] = bhv.enabled();

        for (const auto &body : bhv)
            node["Bodies"].push_back(body->meta.index);
        node["Bodies"].SetStyle(YAML::EmitterStyle::Flow);
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::behaviour2D &bhv)
    {
        if (!node.IsMap() || node.size() < 2)
            return false;
        bhv.clear();

        bhv.enabled(node["Enabled"].as<bool>());
        for (const YAML::Node &n : node["Bodies"])
            bhv.add(bhv.world.bodies[n.as<std::size_t>()]);
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::collider2D::specs::properties>
{
    static YAML::Node encode(const ppx::collider2D::specs::properties &props)
    {
        YAML::Node node;
        node["Density"] = props.density;
        node["Charge density"] = props.charge_density;
        node["Restitution"] = props.restitution;
        node["Friction"] = props.friction;
        node["Shape"] = (int)props.shape;
        switch (props.shape)
        {
        case ppx::collider2D::stype::CIRCLE:
            node["Radius"] = props.radius;
            break;
        case ppx::collider2D::stype::POLYGON:
            for (const glm::vec2 &v : props.vertices)
                node["Vertices"].push_back(v);
        }
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::collider2D::specs::properties &props)
    {
        if (!node.IsMap() || node.size() < 5)
            return false;

        props.density = node["Density"].as<float>();
        props.charge_density = node["Charge density"].as<float>();
        props.restitution = node["Restitution"].as<float>();
        props.friction = node["Friction"].as<float>();
        props.shape = (ppx::collider2D::stype)node["Shape"].as<int>();
        if (node["Radius"])
        {
            props.radius = node["Radius"].as<float>();
            props.shape = ppx::collider2D::stype::CIRCLE;
        }
        else if (node["Vertices"])
        {
            props.vertices.clear();
            for (const YAML::Node &n : node["Vertices"])
                props.vertices.push_back(n.as<glm::vec2>());
            props.shape = ppx::collider2D::stype::POLYGON;
        }
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
        node["Properties"] = collider.props;

        return node;
    }
    static bool decode(const YAML::Node &node, ppx::collider2D::specs &collider)
    {
        if (!node.IsMap() || node.size() != 3)
            return false;

        collider.position = node["Position"].as<glm::vec2>();
        collider.rotation = node["Rotation"].as<float>();
        collider.props = node["Properties"].as<ppx::collider2D::specs::properties>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::body2D::specs::properties>
{
    static YAML::Node encode(const ppx::body2D::specs::properties &props)
    {
        YAML::Node node;
        node["Mass"] = props.mass;
        node["Charge"] = props.charge;
        node["Type"] = (int)props.type;
        for (const ppx::collider2D::specs &collider : props.colliders)
            node["Colliders"].push_back(collider);
        return node;
    }

    static bool decode(const YAML::Node &node, ppx::body2D::specs::properties &props)
    {
        if (!node.IsMap() || node.size() < 3)
            return false;

        props.mass = node["Mass"].as<float>();
        props.charge = node["Charge"].as<float>();
        props.type = (ppx::body2D::btype)node["Type"].as<int>();
        if (node["Colliders"])
            for (const YAML::Node &n : node["Colliders"])
                props.colliders.push_back(n.as<ppx::collider2D::specs>());

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
        node["Properties"] = body.props;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::body2D::specs &body)
    {
        if (!node.IsMap() || node.size() != 5)
            return false;

        body.position = node["Position"].as<glm::vec2>();
        body.velocity = node["Velocity"].as<glm::vec2>();
        body.rotation = node["Rotation"].as<float>();
        body.angular_velocity = node["Angular velocity"].as<float>();
        body.props = node["Properties"].as<ppx::body2D::specs::properties>();

        return true;
    }
};

template <> struct kit::yaml::codec<ppx::specs::joint2D::properties>
{
    static YAML::Node encode(const ppx::specs::joint2D::properties &props)
    {
        YAML::Node node;
        node["Bodies collide"] = props.bodies_collide;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::specs::joint2D::properties &props)
    {
        if (!node.IsMap() || node.size() < 1)
            return false;

        props.bodies_collide = node["Bodies collide"].as<bool>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::specs::constraint2D::properties>
{
    static YAML::Node encode(const ppx::specs::constraint2D::properties &props)
    {
        YAML::Node node = kit::yaml::codec<ppx::specs::joint2D::properties>::encode(props);
        node["Is soft"] = props.is_soft;
        node["Frequency"] = props.frequency;
        node["Damping ratio"] = props.damping_ratio;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::specs::constraint2D::properties &props)
    {
        if (!kit::yaml::codec<ppx::specs::joint2D::properties>::decode(node, props))
            return false;
        if (!node.IsMap() || node.size() < 4)
            return false;

        props.is_soft = node["Is soft"].as<bool>();
        props.frequency = node["Frequency"].as<float>();
        props.damping_ratio = node["Damping ratio"].as<float>();

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

        return node;
    }
    static bool decode(const YAML::Node &node, ppx::specs::joint2D &joint)
    {
        if (!node.IsMap() || node.size() != 2)
            return false;

        if (node["Index1"])
            joint.bindex1 = node["Index1"].as<std::size_t>();
        else
            joint.bspecs1 = node["Body1"].as<ppx::body2D::specs>();

        if (node["Index2"])
            joint.bindex2 = node["Index2"].as<std::size_t>();
        else
            joint.bspecs2 = node["Body2"].as<ppx::body2D::specs>();

        return true;
    }
};

template <> struct kit::yaml::codec<ppx::rotor_joint2D::specs::properties>
{
    static YAML::Node encode(const ppx::rotor_joint2D::specs::properties &props)
    {
        YAML::Node node = kit::yaml::codec<ppx::specs::constraint2D::properties>::encode(props);
        node["Torque"] = props.torque;
        node["Correction factor"] = props.correction_factor;
        node["Target speed"] = props.target_speed;
        node["Min angle"] = props.min_angle;
        node["Max angle"] = props.max_angle;
        node["Spin indefinitely"] = props.spin_indefinitely;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::rotor_joint2D::specs::properties &props)
    {
        if (!kit::yaml::codec<ppx::specs::constraint2D::properties>::decode(node, props))
            return false;
        if (!node.IsMap() || node.size() != 10)
            return false;

        props.torque = node["Torque"].as<float>();
        props.correction_factor = node["Correction factor"].as<float>();
        props.target_speed = node["Target speed"].as<float>();
        props.min_angle = node["Min angle"].as<float>();
        props.max_angle = node["Max angle"].as<float>();
        props.spin_indefinitely = node["Spin indefinitely"].as<bool>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::rotor_joint2D::specs>
{
    static YAML::Node encode(const ppx::rotor_joint2D::specs &rj)
    {
        YAML::Node node;
        node["Joint"] = kit::yaml::codec<ppx::specs::joint2D>::encode(rj);
        node["Properties"] = rj.props;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::rotor_joint2D::specs &rj)
    {
        if (!node.IsMap() || node.size() != 2)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], rj))
            return false;
        rj.props = node["Properties"].as<ppx::rotor_joint2D::specs::properties>();

        return true;
    }
};

template <> struct kit::yaml::codec<ppx::motor_joint2D::specs::properties>
{
    static YAML::Node encode(const ppx::motor_joint2D::specs::properties &props)
    {
        YAML::Node node = kit::yaml::codec<ppx::specs::constraint2D::properties>::encode(props);
        node["Force"] = props.force;
        node["Correction factor"] = props.correction_factor;
        node["Target speed"] = props.target_speed;
        node["Target offset"] = props.target_offset;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::motor_joint2D::specs::properties &props)
    {
        if (!kit::yaml::codec<ppx::specs::constraint2D::properties>::decode(node, props))
            return false;
        if (!node.IsMap() || node.size() != 8)
            return false;

        props.force = node["Force"].as<float>();
        props.correction_factor = node["Correction factor"].as<float>();
        props.target_speed = node["Target speed"].as<float>();
        props.target_offset = node["Target offset"].as<glm::vec2>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::motor_joint2D::specs>
{
    static YAML::Node encode(const ppx::motor_joint2D::specs &mj)
    {
        YAML::Node node;
        node["Joint"] = kit::yaml::codec<ppx::specs::joint2D>::encode(mj);
        node["Properties"] = mj.props;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::motor_joint2D::specs &mj)
    {
        if (!node.IsMap() || node.size() != 2)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], mj))
            return false;
        mj.props = node["Properties"].as<ppx::motor_joint2D::specs::properties>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::distance_joint2D::specs::properties>
{
    static YAML::Node encode(const ppx::distance_joint2D::specs::properties &props)
    {
        YAML::Node node = kit::yaml::codec<ppx::specs::constraint2D::properties>::encode(props);
        node["Min distance"] = props.min_distance;
        node["Max distance"] = props.max_distance;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::distance_joint2D::specs::properties &props)
    {
        if (!kit::yaml::codec<ppx::specs::constraint2D::properties>::decode(node, props))
            return false;
        if (!node.IsMap() || node.size() != 6)
            return false;

        props.min_distance = node["Min distance"].as<float>();
        props.max_distance = node["Max distance"].as<float>();
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
        node["Deduce distance"] = dj.deduce_distance;
        node["Properties"] = dj.props;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::distance_joint2D::specs &dj)
    {
        if (!node.IsMap() || node.size() != 5)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], dj))
            return false;
        dj.ganchor1 = node["Anchor1"].as<glm::vec2>();
        dj.ganchor2 = node["Anchor2"].as<glm::vec2>();
        dj.deduce_distance = node["Deduce distance"].as<bool>();
        dj.props = node["Properties"].as<ppx::distance_joint2D::specs::properties>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::ball_joint2D::specs::properties>
{
    static YAML::Node encode(const ppx::ball_joint2D::specs::properties &props)
    {
        YAML::Node node = kit::yaml::codec<ppx::specs::constraint2D::properties>::encode(props);
        node["Min angle"] = props.min_angle;
        node["Max angle"] = props.max_angle;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::ball_joint2D::specs::properties &props)
    {
        if (!kit::yaml::codec<ppx::specs::constraint2D::properties>::decode(node, props))
            return false;
        if (!node.IsMap() || node.size() != 6)
            return false;

        props.min_angle = node["Min angle"].as<float>();
        props.max_angle = node["Max angle"].as<float>();
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
        node["Properties"] = bj.props;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::ball_joint2D::specs &bj)
    {
        if (!node.IsMap() || node.size() != 3)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], bj))
            return false;
        bj.deduce_angle = node["Deduce angle"].as<bool>();
        bj.props = node["Properties"].as<ppx::ball_joint2D::specs::properties>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::prismatic_joint2D::specs::properties>
{
    static YAML::Node encode(const ppx::prismatic_joint2D::specs::properties &props)
    {
        YAML::Node node = kit::yaml::codec<ppx::specs::constraint2D::properties>::encode(props);
        node["Axis"] = props.axis;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::prismatic_joint2D::specs::properties &props)
    {
        if (!kit::yaml::codec<ppx::specs::constraint2D::properties>::decode(node, props))
            return false;
        if (!node.IsMap() || node.size() != 5)
            return false;

        props.axis = node["Axis"].as<glm::vec2>();
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
        node["Properties"] = pj.props;
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
        pj.props = node["Properties"].as<ppx::prismatic_joint2D::specs::properties>();
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
        node["Properties"] = revj.props;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::revolute_joint2D::specs &revj)
    {
        if (!node.IsMap() || node.size() != 3)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], revj))
            return false;
        revj.ganchor = node["Anchor"].as<glm::vec2>();
        revj.props = node["Properties"].as<ppx::revolute_joint2D::specs::properties>();
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
        node["Properties"] = weldj.props;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::weld_joint2D::specs &weldj)
    {
        if (!node.IsMap() || node.size() != 3)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], weldj))
            return false;
        weldj.ganchor = node["Anchor"].as<glm::vec2>();
        weldj.props = node["Properties"].as<ppx::weld_joint2D::specs::properties>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::spring_joint2D::specs::properties>
{
    static YAML::Node encode(const ppx::spring_joint2D::specs::properties &props)
    {
        YAML::Node node = kit::yaml::codec<ppx::specs::joint2D::properties>::encode(props);
        node["Frequency"] = props.frequency;
        node["Damping ratio"] = props.damping_ratio;
        node["Min length"] = props.min_length;
        node["Max length"] = props.max_length;
        node["Non linear terms"] = props.non_linear_terms;
        node["Non linear contribution"] = props.non_linear_contribution;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::spring_joint2D::specs::properties &props)
    {
        if (!kit::yaml::codec<ppx::specs::joint2D::properties>::decode(node, props))
            return false;
        if (!node.IsMap() || node.size() != 7)
            return false;

        props.frequency = node["Frequency"].as<float>();
        props.damping_ratio = node["Damping ratio"].as<float>();
        props.min_length = node["Min length"].as<float>();
        props.max_length = node["Max length"].as<float>();
        props.non_linear_terms = node["Non linear terms"].as<std::uint32_t>();
        props.non_linear_contribution = node["Non linear contribution"].as<float>();
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
        node["Properties"] = sp.props;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::spring_joint2D::specs &sp)
    {
        if (!node.IsMap() || node.size() != 4)
            return false;

        if (!kit::yaml::codec<ppx::specs::joint2D>::decode(node["Joint"], sp))
            return false;
        sp.ganchor1 = node["Anchor1"].as<glm::vec2>();
        sp.ganchor2 = node["Anchor2"].as<glm::vec2>();
        sp.props = node["Properties"].as<ppx::spring_joint2D::specs::properties>();
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
            node["Behaviours"][bhv->id()] = *bhv;
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
            node["Managers"][mng->id()] = *mng;
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
        node["Enabled"] = cm.enabled();
        YAML::Node nbroad = node["Broad"];

        nbroad["Multithreading"] = cm.broad()->params.multithreading;
        nbroad["Name"] = cm.broad()->name();
        if (cm.broad<ppx::brute_force_broad2D>())
            nbroad["Method"] = 0;
        else if (auto broad = cm.broad<ppx::quad_tree_broad2D>())
        {
            nbroad["Method"] = 1;
            nbroad["Force square"] = broad->force_square_shape;
            nbroad["Rebuild time threshold"] = broad->rebuild_time_threshold;

            const auto &props = broad->quad_tree().props();
            nbroad["Max colliders"] = props.elements_per_quad;
            nbroad["Max depth"] = props.max_depth;
            nbroad["Min size"] = props.min_quad_size;
        }
        else if (cm.broad<ppx::sort_sweep_broad2D>())
            nbroad["Method"] = 2;

        YAML::Node nnarrow = node["Narrow"];
        nnarrow["Name"] = cm.narrow()->name();
        if (auto narrow = cm.narrow<ppx::gjk_epa_narrow2D>())
        {
            nnarrow["Method"] = 0;
            nnarrow["EPA Threshold"] = narrow->epa_threshold;
        }
        else if (cm.narrow<ppx::sat_narrow2D>())
            nnarrow["Method"] = 1;

        YAML::Node nsolv = node["Contacts"];
        nsolv["Contact lifetime"] = cm.contact_solver()->params.contact_lifetime;
        if (auto colsolv = cm.contact_solver<ppx::contact_solver2D<ppx::nonpen_contact2D>>())
            nsolv["Solver method"] = 0;
        else if (auto colsolv = cm.contact_solver<ppx::contact_solver2D<ppx::spring_contact2D>>())
            nsolv["Solver method"] = 1;

        nsolv["Rigidity"] = ppx::spring_contact2D::rigidity;
        nsolv["Max normal damping"] = ppx::spring_contact2D::max_normal_damping;
        nsolv["Max tangent damping"] = ppx::spring_contact2D::max_tangent_damping;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::collision_manager2D &cm)
    {
        if (!node.IsMap() || node.size() != 4)
            return false;

        cm.enabled(node["Enabled"].as<bool>());
        const YAML::Node nbroad = node["Broad"];

        cm.broad()->params.multithreading = nbroad["Multithreading"].as<bool>();
        if (nbroad["Method"])
        {
            const int method = nbroad["Method"].as<int>();
            if (method == 0)
                cm.set_broad<ppx::brute_force_broad2D>();
            else if (method == 1)
            {
                auto qtbroad = cm.set_broad<ppx::quad_tree_broad2D>();
                qtbroad->force_square_shape = nbroad["Force square"].as<bool>();
                qtbroad->rebuild_time_threshold = nbroad["Rebuild time threshold"].as<float>();

                auto props = qtbroad->quad_tree().props();
                props.elements_per_quad = nbroad["Max colliders"].as<std::size_t>();
                props.max_depth = nbroad["Max depth"].as<std::uint32_t>();
                props.min_quad_size = nbroad["Min size"].as<float>();
                qtbroad->quad_tree().props(props);
            }
            else if (method == 2)
                cm.set_broad<ppx::sort_sweep_broad2D>();
        }

        YAML::Node nnarrow = node["Narrow"];
        if (nnarrow["Method"])
        {
            const int method = nnarrow["Method"].as<int>();
            if (method == 0)
                cm.set_narrow<ppx::gjk_epa_narrow2D>(nnarrow["EPA Threshold"].as<float>());
            else if (method == 1)
                cm.set_narrow<ppx::sat_narrow2D>();
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
        cm.contact_solver()->params.contact_lifetime = nsolv["Contact lifetime"].as<std::uint32_t>();

        ppx::spring_contact2D::rigidity = nsolv["Rigidity"].as<float>();
        ppx::spring_contact2D::max_normal_damping = nsolv["Max normal damping"].as<float>();
        ppx::spring_contact2D::max_tangent_damping = nsolv["Max tangent damping"].as<float>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::specs::joint_manager2D::constraints2D>
{
    static YAML::Node encode(const ppx::specs::joint_manager2D::constraints2D &params)
    {
        YAML::Node node;
        node["Velocity iterations"] = params.velocity_iterations;
        node["Position iterations"] = params.position_iterations;
        node["Warmup"] = params.warmup;
        node["Baumgarte correction"] = params.baumgarte_correction;
        node["Baumgarte coef"] = params.baumgarte_coef;
        node["Baumgarte threshold"] = params.baumgarte_threshold;
        node["Slop"] = params.slop;
        node["Max position correction"] = params.max_position_correction;
        node["Position resolution speed"] = params.position_resolution_speed;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::specs::joint_manager2D::constraints2D &params)
    {
        if (!node.IsMap() || node.size() != 9)
            return false;

        params.velocity_iterations = node["Velocity iterations"].as<std::uint32_t>();
        params.position_iterations = node["Position iterations"].as<std::uint32_t>();
        params.warmup = node["Warmup"].as<bool>();
        params.baumgarte_correction = node["Baumgarte correction"].as<bool>();
        params.baumgarte_coef = node["Baumgarte coef"].as<float>();
        params.baumgarte_threshold = node["Baumgarte threshold"].as<float>();
        params.slop = node["Slop"].as<float>();
        params.max_position_correction = node["Max position correction"].as<float>();
        params.position_resolution_speed = node["Position resolution speed"].as<float>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::specs::island_manager2D>
{
    static YAML::Node encode(const ppx::specs::island_manager2D &params)
    {
        YAML::Node node;
        node["Enable split"] = params.enable_split;
        node["Enable sleep"] = params.enable_sleep;
        node["Multithreading"] = params.multithreading;
        node["Lower sleep energy threshold"] = params.lower_sleep_energy_threshold;
        node["Upper sleep energy threshold"] = params.upper_sleep_energy_threshold;
        node["Body count mid threshold reference"] = params.body_count_mid_threshold_reference;
        node["Sleep time threshold"] = params.sleep_time_threshold;
        return node;
    }
    static bool decode(const YAML::Node &node, ppx::specs::island_manager2D &params)
    {
        if (!node.IsMap() || node.size() != 8)
            return false;

        params.enable_split = node["Enable split"].as<bool>();
        params.enable_sleep = node["Enable sleep"].as<bool>();
        params.multithreading = node["Multithreading"].as<bool>();
        params.lower_sleep_energy_threshold = node["Lower sleep energy threshold"].as<float>();
        params.upper_sleep_energy_threshold = node["Upper sleep energy threshold"].as<float>();
        params.body_count_mid_threshold_reference = node["Body count mid threshold reference"].as<std::uint32_t>();
        params.sleep_time_threshold = node["Sleep time threshold"].as<float>();
        return true;
    }
};

template <> struct kit::yaml::codec<ppx::world2D>
{
    static YAML::Node encode(const ppx::world2D &world)
    {
        YAML::Node node;
        node["Constraint settings"] = world.joints.constraints.params;
        node["Island settings"] = world.islands.params;
        node["Island settings"]["Enabled"] = world.islands.enabled();
        node["Behaviour manager"] = world.behaviours;
        node["Collision manager"] = world.collisions;

        node["Semi-implicit integration"] = world.semi_implicit_integration;
        node["Integrator"] = world.integrator;
        node["Joints repository"] = world.joints;
        node["Bounding box enlargement"] = world.colliders.params.bbox_enlargement;
        node["Body manager"] = world.bodies;

        return node;
    }
    static bool decode(const YAML::Node &node, ppx::world2D &world)
    {
        if (!node.IsMap() || node.size() != 8)
            return false;

        world.semi_implicit_integration = node["Semi-implicit integration"].as<bool>();
        world.colliders.params.bbox_enlargement = node["Bounding box enlargement"].as<float>();
        node["Body manager"].as<ppx::body_manager2D>(world.bodies);
        node["Integrator"].as<rk::integrator<float>>(world.integrator);
        node["Behaviour manager"].as<ppx::behaviour_manager2D>(world.behaviours);
        node["Collision manager"].as<ppx::collision_manager2D>(world.collisions);
        node["Joints repository"].as<ppx::joint_repository2D>(world.joints);

        world.joints.constraints.params = node["Constraint settings"].as<ppx::specs::joint_manager2D::constraints2D>();
        world.islands.params = node["Island settings"].as<ppx::specs::island_manager2D>();
        world.islands.enabled(node["Island settings"]["Enabled"].as<bool>());

        return true;
    }
};