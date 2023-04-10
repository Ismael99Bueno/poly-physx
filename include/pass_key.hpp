#ifndef PASS_KEY_HPP
#define PASS_KEY_HPP

namespace ppx
{
    class engine_key final
    {
        engine_key() = default;
        engine_key(const engine_key &) = delete;
        engine_key &operator=(const engine_key &) = delete;
        friend class engine2D;
    };
    class collider_key final
    {
        collider_key() = default;
        // collider_key(const collider_key &) = delete;
        // collider_key &operator=(const collider_key &) = delete;
        friend class collider2D;
        friend class quad_tree2D;
    };
}

#endif