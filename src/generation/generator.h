#ifndef GENERATOR_H
#define GENERATOR_H

#include <queue>
#include <random>

#include "../types.h"
#include "tensor_field.h"
#include "road_storage.h"


enum IntegrationStatus {
    Continue,
    Terminate,
    Abort
};


struct Integration {
    IntegrationStatus status;
    std::optional<DVector2> delta;
    DVector2 integration_front;
    bool negate; 
    std::list<DVector2> points;

    Integration(DVector2 seed, bool negate) :
        status(Continue),
        integration_front(seed),
        negate(negate),
        points({seed})
    {}
};


struct GeneratorParameters {
    int max_seed_retries;
    int max_integration_iterations;
    double d_sep;
    double d_sep2;
    double d_test;
    double d_test2;
    double d_circle;
    double d_circle2;
    double dl;
    double dl2;
    double d_lookahead;
    double theta_max; // maximum streamline joining angle
    double epsilon;
    double node_sep;
    double node_sep2;


    GeneratorParameters(
        int max_seed_retries,
        int max_integration_iterations,
        double d_sep,
        double d_test,
        double d_circle,
        double dl,
        double d_lookahead,
        double theta_max,
        double epsilon,
        double node_sep
    );
};


class RoadGenerator : public RoadStorage {
    private:
        using seed_queue = std::queue<DVector2>;
        static constexpr int kQuadTreeDepth = 10; // area of 3 pixels at 1920x1080
        static constexpr int kQuadTreeLeafCapacity = 10;

        GeneratorParameters* params_;
        std::array<seed_queue, Eigenfield::count> seeds_;
        std::default_random_engine gen_;
        std::uniform_real_distribution<double> dist_;

        TensorField* field_;

        int tangent_samples_ = 5;
        Box<double> viewport_;

        bool in_bounds(const DVector2& p) const;

        void add_candidate_seed(DVector2 pos, Eigenfield ef);

        std::optional<DVector2> get_seed(size_t road_type, Eigenfield ef);

        DVector2 get_eigenvector(const DVector2& x, const Eigenfield& ef) const;
        DVector2 integrate_rk4(const DVector2& x, const Eigenfield& ef, const double& dl) const;

        void extend_road(Integration& res, const size_t& road_type, const Eigenfield& ef) const;

        std::list<DVector2>
        spawn_road(size_t road_type, DVector2 seed_point, Eigenfield ef);

        int generate_roads(size_t road_type);

        
        void simplify_streamline(size_t road_type, std::list<DVector2>& points) const;
        void douglas_peucker(
            const double& epsilon,
            const double& min_sep2,
            std::list<DVector2>& points,
            std::list<DVector2>::iterator begin,
            std::list<DVector2>::iterator end
        ) const;


        void push_road(std::list<DVector2>& points, size_t road_type, Eigenfield ef);

        DVector2 tangent(const NodeHandle& handle) const;

        std::optional<NodeHandle> joining_candidate(const NodeHandle& handle) const;
        std::list<DVector2> joining_streamline(double dl, DVector2 x0, DVector2 x1) const;
        void connect_roads(size_t road, Eigenfield ef);


    public:
        RoadGenerator(
                TensorField* field,
                size_t road_type_count,
                GeneratorParameters* params,
                Box<double> viewport
            );

        size_t road_type_count() const;
        void reset(Box<double> new_viewport);
        void clear();
        void generate();
};
#endif
