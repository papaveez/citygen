#pragma once


#include "../types.h"
#include "tensor_field.h"


enum Direction : size_t {
    Minor,
    Major,
    EigenfieldCount
};


class NumericalFieldIntegrator {
private:
    TensorField* field_;

protected:
    DVector2 get_vector(const DVector2& x, const Direction& dir) const;

public:
    NumericalFieldIntegrator(TensorField* field);
    virtual ~NumericalFieldIntegrator() = default;

    virtual DVector2 
    integrate(
        const DVector2& x, 
        const Direction& d, 
        const double& dl
    ) const = 0;
};


class RK4 : public NumericalFieldIntegrator {
public:
    RK4(TensorField* _field);

    DVector2 
    integrate(
        const DVector2& x, 
        const Direction& d, 
        const double& dl
    ) const override;
};
