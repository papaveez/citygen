#pragma once

#include <cstddef>
#include <limits>
#include <variant>
#include <vector>

#include "raylib.h"

static constexpr float d_epsilon = std::numeric_limits<float>::epsilon();

struct Tensor {
    // 2x2 symmetric, traceless matrix represented as
    // R * | cos(2θ)  sin(2θ) | --> | a  b |
    //     | sin(2θ) -cos(2θ) |     | _  _ |
    float a;
    float b;
    float r;
    float theta;

    static Tensor degenerate();
    static Tensor from_a_b(float a, float b);
    static Tensor from_r_theta(float r, float theta);
    static Tensor from_xy(const Vector2& xy);

    void set_r_theta();

    bool is_degenerate() const;
    Vector2 get_major_eigenvector() const;
    Vector2 get_minor_eigenvector() const;

    Tensor rotate(float angle) const;

    Tensor operator+(const Tensor& other) const;

    // right scalar mult
    Tensor operator*(float right) const;


    // left scalar mult
    friend Tensor operator*(float left, const Tensor& right);
};


class BasisField {
    protected:
        Vector2 centre_;
        float size_;
        float decay_;

        virtual Tensor get_tensor(const Vector2& pos) const;
        float get_tensor_weight(const Vector2& pos) const;

    public:
        BasisField(Vector2 centre);
        BasisField(Vector2 centre, float size, float decay);
        virtual ~BasisField() = default;

        const Vector2& get_centre() const;
        const float& get_size() const;
        const float& get_decay() const;

        void set_centre(Vector2 centre);
        void set_size(float size);
        void set_decay(float decay);


        Tensor get_weighted_tensor(const Vector2& pos) const;
};


class Grid : public BasisField {
    private:
        float theta;


    public:
        Grid(float theta, Vector2 centre);
        Grid(float theta, Vector2 centre, float size, float decay);

        Tensor get_tensor(const Vector2& pos) const override;
        void set_theta(float _theta);
};


class Radial : public BasisField {
    public:
        Radial(Vector2 centre);
        Radial(Vector2 centre, float size, float decay);

        Tensor get_tensor(const Vector2& pos) const override;
};


class TensorField {
private:
    std::vector<std::variant<Grid, Radial>> basis_fields;

public:
    TensorField();

    template <typename V>
    void add_basis(V&& basis) {
        basis_fields.push_back(std::move(basis));
    }

    const Vector2& get_centre(size_t idx) const;
    const float& get_size(size_t idx) const;
    const float& get_decay(size_t idx) const;


    void set_centre(size_t idx, Vector2 centre);
    void set_size(size_t idx, float size);
    void set_decay(size_t idx, float decay);

    void erase(size_t idx);

    template<typename V>
    bool is(size_t idx) const {
        if (idx >= basis_fields.size()) return false;

        if (const V* ptr = std::get_if<V>(&basis_fields[idx])) {
            return true;
        }

        return false;
    }

    template<typename V, typename Func>
    void visit_if(size_t idx, Func&& func) {
        if (idx >= basis_fields.size())
            return;

        if (V* ptr = std::get_if<V>(&basis_fields[idx]))
            std::invoke(std::forward<Func>(func), *ptr);
    }


    template<typename V, typename Func>
    void visit_if(size_t idx, Func&& func) const {
        if (idx >= basis_fields.size())
            return;

        if (const V* ptr = std::get_if<V>(&basis_fields[idx]))
            std::invoke(std::forward<Func>(func), *ptr);
    }

    Tensor sample(const Vector2& pos) const;
    size_t size() const;
    void clear();
};
