#include "force2D.hpp"

namespace phys
{

    float force2D::energy(const phys::entity2D &e) const { return e.kinetic_energy() + potential_energy(e); }
}