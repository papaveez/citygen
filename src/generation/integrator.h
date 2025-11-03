#pragma once


#include "../types.h"
#include "tensor_field.h"



using ef_mask = unsigned char;

struct Eigenfield {
    enum class EigenDirection : size_t {
        Minor,
        Major,
        Count
    };

    EigenDirection value;

    constexpr Eigenfield(EigenDirection v) : value(v) {}


    static constexpr Eigenfield major() {
        return Eigenfield(EigenDirection::Major);
    };

    static constexpr Eigenfield minor(){
        return Eigenfield(EigenDirection::Minor);
    }

    static constexpr size_t count = static_cast<size_t>(EigenDirection::Count);

    constexpr Eigenfield opposite() {
        if (value == EigenDirection::Major) {
            return minor();
        } else {
            return major();
        }
    }


    constexpr operator size_t() const {return static_cast<size_t>(value);}
    constexpr operator ef_mask() const {return 1<<size_t(value);}
    constexpr ef_mask mask() const {return 1<<size_t(value);}


    constexpr bool operator == (const Eigenfield& other) const {
        return value == other.value;
    }


    constexpr ef_mask operator | (const Eigenfield& other) const {
        return static_cast<ef_mask>(other) | static_cast<ef_mask>(*this);
    }
};


class NumericalFieldIntegrator {
private:
    TensorField* field_;

protected:
    DVector2 get_vector(const DVector2& x, const Eigenfield& ef) const;

public:
    NumericalFieldIntegrator(TensorField* field);
    virtual ~NumericalFieldIntegrator() = default;

    virtual DVector2 
    integrate(
        const DVector2& x, 
        const Eigenfield& d, 
        const double& dl
    ) const = 0;
};


class RK4 : public NumericalFieldIntegrator {
public:
    RK4(TensorField* _field);

    DVector2 
    integrate(
        const DVector2& x, 
        const Eigenfield& d, 
        const double& dl
    ) const override;
};
