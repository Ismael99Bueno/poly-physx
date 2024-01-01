#include "ppx/internal/pch.hpp"
#include "ppx/collision/resolution/collision_resolution2D.hpp"

namespace ppx
{
void collision_resolution2D::inherit(collision_resolution2D &colres)
{
    multithreaded = colres.multithreaded;
}
} // namespace ppx