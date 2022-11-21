#ifndef CONSTRAIN_INTERFACE_HPP
#define CONSTRAIN_INTERFACE_HPP

#include "entity_ptr.hpp"
#include <vector>
#include <array>

#define POS_PER_ENTITY 3

namespace phys
{
    class constrain_interface
    {
    public:
        virtual float value() const = 0;

    private:
        std::array<float, POS_PER_ENTITY> constrain_grad(const entity_ptr &e) const;
        std::array<float, POS_PER_ENTITY> constrain_grad_derivative(const entity_ptr &e) const;
        std::array<float, POS_PER_ENTITY> gradient(const entity_ptr &e,
                                                   float (constrain_interface::*constrain)() const) const;

        virtual float derivative() const = 0;
        virtual std::size_t size() const = 0;
        virtual const entity_ptr &operator[](std::size_t index) const = 0;

        friend class compeller2D;
    };
}

#endif