#ifndef PASS_KEY_HPP
#define PASS_KEY_HPP

namespace ppx
{
    class engine_key final
    {
        engine_key() = default;
        friend class enine2D;
    };
    class collider_key final
    {
        collider_key() = default;
        friend class collider2D;
    };
}

#endif