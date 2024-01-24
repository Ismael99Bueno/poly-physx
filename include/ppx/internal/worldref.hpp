#pragma once

namespace ppx
{
template <typename T> class worldref
{
  public:
    T &world;

    worldref(T &world) : world(world)
    {
    }
    worldref(const worldref &other) : world(other.world)
    {
    }
    worldref(worldref &&other) : world(other.world)
    {
    }

    worldref &operator=(const worldref &other)
    {
        return *this;
    }
    worldref &operator=(worldref &&other)
    {
        return *this;
    }
};

class world2D;
using worldref2D = worldref<world2D>;
} // namespace ppx