#include "constrain_interface.hpp"
#include <algorithm>

#define TOLERANCE 1.e-2f

namespace phys
{
    std::array<float, 3> constrain_interface::constrain_grad(const entity_ptr &e) const { return gradient(e, &constrain_interface::value); }
    std::array<float, 3> constrain_interface::constrain_grad_derivative(const entity_ptr &e) const { return gradient(e, &constrain_interface::derivative); }
    std::array<float, 3> constrain_interface::gradient(const entity_ptr &e,
                                                       float (constrain_interface::*constrain)() const) const
    {
        const float x = e->pos().x;
        e->translate({-TOLERANCE, 0.f});
        const float cx1 = (this->*constrain)();
        e->translate({2.f * TOLERANCE, 0.f});
        const float cx2 = (this->*constrain)();
        e->pos({x, e->pos().y});

        const float y = e->pos().y;
        e->translate({0.f, -TOLERANCE});
        const float cy1 = (this->*constrain)();
        e->translate({0.f, 2.f * TOLERANCE});
        const float cy2 = (this->*constrain)();
        e->pos({e->pos().x, y});

        const float angpos = e->angpos();
        e->angpos(e->angpos() - TOLERANCE);
        const float ca1 = (this->*constrain)();
        e->angpos(e->angpos() + 2.f * TOLERANCE);
        const float ca2 = (this->*constrain)();
        e->angpos(angpos);

        return {(cx2 - cx1) / (2.f * TOLERANCE), (cy2 - cy1) / (2.f * TOLERANCE), (ca2 - ca1) / (2.f * TOLERANCE)};
    }
}