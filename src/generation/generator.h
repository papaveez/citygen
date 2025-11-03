#ifndef GENERATOR_H
#define GENERATOR_H

#include <queue>
#include <vector>
#include <random>
#include <unordered_map>

#include "../types.h"
#include "integrator.h"
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


        std::unique_ptr<NumericalFieldIntegrator> integrator_;
        std::vector<RoadType> road_types_;
        std::unordered_map<RoadType, GeneratorParameters> params_;
        std::array<seed_queue, Eigenfield::count> seeds_;
        std::default_random_engine gen_;
        std::uniform_real_distribution<double> dist_;

        int tangent_samples_ = 5;
        Box<double> viewport_;

        bool in_bounds(const DVector2& p) const;


        void add_candidate_seed(DVector2 pos, Eigenfield ef);

        std::optional<DVector2> get_seed(RoadType road, Eigenfield ef);


        void extend_road(
            Integration& res,
            const RoadType& road,
            const Eigenfield& ef
        ) const;

        std::list<DVector2>
        generate_road(RoadType road, DVector2 seed_point, Eigenfield ef);

        int generate_all_roads(RoadType road);

        
        void simplify_streamline(RoadType road, std::list<DVector2>& points) const;
        void douglas_peucker(
            const double& epsilon,
            const double& min_sep2,
            std::list<DVector2>& points,
            std::list<DVector2>::iterator begin,
            std::list<DVector2>::iterator end
        ) const;


        void push_road(std::list<DVector2>& points, RoadType road, Eigenfield ef);

        DVector2 tangent(const NodeHandle& handle) const;

        std::optional<NodeHandle> joining_candidate(const NodeHandle& handle) const;
        std::list<DVector2> joining_streamline(double dl, DVector2 x0, DVector2 x1) const;
        void connect_roads(RoadType road, Eigenfield ef);


    public:
        RoadGenerator(
                std::unique_ptr<NumericalFieldIntegrator>& integrator,
                std::unordered_map<RoadType, GeneratorParameters>,
                Box<double> viewport
            );

        // getters
        const std::vector<RoadType>& get_road_types() const;
        const std::unordered_map<RoadType, GeneratorParameters>& get_parameters() const;


        void reset(Box<double> new_viewport);
        void clear();
        void generate();
};
#endif
