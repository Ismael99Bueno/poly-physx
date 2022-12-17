#ifndef CONSTRAINT_INTERFACE_HPP
#define CONSTRAINT_INTERFACE_HPP

#include "entity2D.hpp"
#include <vector>
#include <array>
#include <functional>

namespace phys
{
    class constraint_interface
    {
    public:
        virtual float value() const = 0;
        float stiffness() const;
        float dampening() const;

        void stiffness(float stiffness);
        void dampening(float dampening);

        virtual bool try_validate() = 0;

    private:
        float m_stiffness = 50.f, m_dampening = 10.f;
        std::array<float, 3> constraint_grad(entity2D &e) const;
        std::array<float, 3> constraint_grad_derivative(entity2D &e) const;
        using constraint_fun = std::function<float(const constraint_interface &)>;
        std::array<float, 3> gradient(entity2D &e,
                                      const constraint_fun &constraint) const;

        virtual float derivative() const = 0;
        virtual std::size_t size() const = 0;
        virtual entity2D &operator[](std::size_t index) const = 0;

        friend class compeller2D;
    };
}

#endif