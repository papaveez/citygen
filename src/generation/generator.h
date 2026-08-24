#ifndef GENERATOR_H
#define GENERATOR_H

#include <queue>
#include <random>

#include "../util.h"
#include "tensor_field.h"
#include "road_storage.h"


enum IntegrationStatus {
    Continue,
    Terminate,
    Abort
};


struct Integration {
    IntegrationStatus status;
    std::optional<Vector2> delta;
    Vector2 integration_front;
    bool negate; 
    std::list<Vector2> points;

    Integration(Vector2 seed, bool negate) :
        status(Continue),
        integration_front(seed),
        negate(negate),
        points({seed})
    {}
};


struct GeneratorParameters {
    int max_seed_retries;
    int max_integration_iterations;
    float d_sep;
    float d_sep2;
    float d_test;
    float d_test2;
    float d_circle;
    float d_circle2;
    float dl;
    float dl2;
    float d_lookahead;
    float theta_max; // maximum streamline joining angle
    float epsilon;
    float node_sep;
    float node_sep2;


    GeneratorParameters(
        int max_seed_retries,
        int max_integration_iterations,
        float d_sep,
        float d_test,
        float d_circle,
        float dl,
        float d_lookahead,
        float theta_max,
        float epsilon,
        float node_sep
    );
};


class RoadGenerator : public RoadStorage {
    private:
        using seed_queue = std::queue<Vector2>;
        static constexpr int kQuadTreeDepth = 10; // area of 3 pixels at 1920x1080
        static constexpr int kQuadTreeLeafCapacity = 10;

        GeneratorParameters* params_;
        std::array<seed_queue, Eigenfield::count> seeds_;
        std::default_random_engine gen_;
        std::uniform_real_distribution<float> dist_;

        TensorField* field_;

        int tangent_samples_ = 5;
        Box viewport_;

        bool in_bounds(const Vector2& p) const;

        void add_candidate_seed(Vector2 pos, Eigenfield ef);

        std::optional<Vector2> get_seed(size_t road_type, Eigenfield ef);

        Vector2 get_eigenvector(const Vector2& x, const Eigenfield& ef) const;
        Vector2 integrate_rk4(const Vector2& x, const Eigenfield& ef, const float& dl) const;

        void extend_road(Integration& res, const size_t& road_type, const Eigenfield& ef) const;

        std::list<Vector2>
        spawn_road(size_t road_type, Vector2 seed_point, Eigenfield ef);

        int generate_roads(size_t road_type);

        
        void simplify_streamline(size_t road_type, std::list<Vector2>& points) const;
        void douglas_peucker(
            const float& epsilon,
            const float& min_sep2,
            std::list<Vector2>& points,
            std::list<Vector2>::iterator begin,
            std::list<Vector2>::iterator end
        ) const;


        void push_road(std::list<Vector2>& points, size_t road_type, Eigenfield ef);

        Vector2 tangent(const NodeHandle& handle) const;

        std::optional<NodeHandle> joining_candidate(const NodeHandle& handle) const;
        std::list<Vector2> joining_streamline(float dl, Vector2 x0, Vector2 x1) const;
        void connect_roads(size_t road, Eigenfield ef);


    public:
        RoadGenerator(
                TensorField* field,
                size_t road_type_count,
                GeneratorParameters* params,
                Box viewport
            );

        size_t road_type_count() const;
        void reset(Box new_viewport);
        void clear();
        void generate();
};
#endif
