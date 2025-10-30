#include "generator.h"
#include "integrator.h"
// #include "node_storage.h"

#include <cassert>
#include <iostream>
#include <iterator>
#include <list>

GeneratorParameters::GeneratorParameters(
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
        ) :
    max_seed_retries(max_seed_retries),
    max_integration_iterations(max_integration_iterations),
    d_sep(d_sep),
    d_sep2(d_sep*d_sep),
    d_test(d_test),
    d_test2(d_test*d_test),
    d_circle(d_circle),
    d_circle2(d_circle*d_circle),
    dl(dl),
    dl2(dl*dl),
    d_lookahead(d_lookahead),
    theta_max(theta_max),
    epsilon(epsilon),
    node_sep(node_sep),
    node_sep2(node_sep*node_sep)
{}


//  SECTION: RoadGenerator

bool RoadGenerator::in_bounds(const DVector2& p) const {
    return viewport_.contains(p);
}


void RoadGenerator::add_candidate_seed(DVector2 pos, Eigenfield ef) {
    seeds_[ef].push(pos);
}


std::optional<DVector2> 
RoadGenerator::get_seed(RoadType road, Eigenfield ef) {
    seed_queue& candidate_queue = seeds_[ef];

    DVector2 seed;
    while (!candidate_queue.empty()) {
        DVector2 seed = candidate_queue.front();
        candidate_queue.pop();
        if (!has_nearby_point(seed, params_.at(road).d_sep, ef)) {
            return seed;
        } 
    }


    for (int count=0; count<params_.at(road).max_seed_retries; count++) {
        seed = DVector2 {
            dist_(gen_)*viewport_.width()  + viewport_.min.x,
            dist_(gen_)*viewport_.height() + viewport_.min.y
        };


        if (!has_nearby_point(seed, params_.at(road).d_sep, ef)) {
            return seed;
        } 
    }
    return {};
}


void RoadGenerator::extend_road(
    Integration& res,
    const RoadType& road, 
    const Eigenfield& ef 
) const {
    if (res.status != Continue) {
        res.status = Abort;
        return;
    };

    DVector2 delta = integrator_->integrate(
        res.integration_front, 
        ef, 
        params_.at(road).dl
    );

    if (res.negate) 
        delta = delta*-1.0;

    if (res.delta.has_value() && dot_product(res.delta.value(), delta) < 0) {
        delta = delta*-1.0;
    }

    if (dot_product(delta, delta) < 0.01) {
        res.status = Abort;
        return;
    }

    res.integration_front = res.integration_front + delta;
    res.delta = delta;
    if (!in_bounds(res.integration_front)) {
        res.status = Abort;
        return;
    }

    res.status = Continue;
    if (has_nearby_point(res.integration_front, params_.at(road).d_test, ef)) {
        res.status = Terminate;
    }
}


std::optional<std::list<DVector2>>
RoadGenerator::generate_road(RoadType road, DVector2 seed_point, Eigenfield ef) {
    Integration forward  (seed_point, false);
    Integration backward (seed_point, true );

    // circle logic
    bool points_diverged = false;
    bool join = false; // flag to join endpoints (in a circular road)

    int count = 0;

    while(count<params_.at(road).max_integration_iterations) {
        extend_road(forward,  road, ef);
        extend_road(backward, road, ef);

        if (backward.status == Abort && forward.status == Abort)
            break;

        if (forward.status != Abort) {
            forward.points.push_back(forward.integration_front);
            count++;
        }

        if (backward.status != Abort) {
            backward.points.push_front(backward.integration_front);
            count++;
        }


        DVector2 ends_diff = forward.points.back() - backward.points.front();
        double sep2 = dot_product(ends_diff, ends_diff);

        if (points_diverged && sep2 < params_.at(road).d_circle2) {
            join = true;
            break;
        } else if (!points_diverged && sep2 > params_.at(road).d_circle2) {
            points_diverged = true;
        }
    }

    backward.points.pop_back(); // remove shared start point

    if (join) {
        forward.points.push_back(backward.points.back()); // join up streamlines
    }

    std::list<DVector2> result;

    result.splice(result.end(), backward.points);
    result.splice(result.end(), forward.points);

    if (result.size() < min_streamline_size_) return {};

    return result;
}


int RoadGenerator::generate_all_roads(RoadType road_type) {
    Eigenfield ef(Major);

    std::optional<DVector2> seed = get_seed(road_type, ef);
    int k = 0;
    while (seed.has_value()) {
        std::optional<std::list<DVector2>> new_streamline
            = generate_road(road_type, seed.value(), ef);

        if (new_streamline.has_value()) {
            simplify_road(road_type, new_streamline.value());

            if (new_streamline.value().size() >= min_streamline_size_) {
                push_road(new_streamline.value(), road_type, ef);
                k += 1;
                ef = flip(ef);
            }
        }
        
        seed = get_seed(road_type, ef);
    };

    std::cout << "generated " << k << " roads" << std::endl;

    // connect_roads(road, Major);
    // connect_roads(road, Minor);

    return k;
}


void RoadGenerator::simplify_road(RoadType road, std::list<DVector2>& points) const {
    assert(params_.at(road).epsilon > 0.0);
    douglas_peucker(params_.at(road).epsilon, params_.at(road).node_sep2, points, points.begin(), points.end());
}


void RoadGenerator::douglas_peucker(const double& epsilon, const double& min_sep2, 
        std::list<DVector2>& points,
        std::list<DVector2>::iterator begin, std::list<DVector2>::iterator end) const 
{
    // must be 3> elements 
    int count = 0;
    for (auto it=begin;it!=end && count < 3;++it, ++count);
    if (count < 3) return;

    auto last_elem = std::prev(end);

    const DVector2& first_pos = *begin;
    const DVector2& last_pos  = *last_elem;

    double d_max = 0.0;
    std::list<DVector2>::iterator index;


    for (auto it=std::next(begin); it != last_elem; ++it) {
        double d = perpendicular_distance(*it, first_pos, last_pos);

        if (d > d_max) {
            d_max = d;
            index = it;
        }
    }

    if (d_max > epsilon) {
        douglas_peucker(epsilon, min_sep2, points, begin, std::next(index));
        douglas_peucker(epsilon, min_sep2, points, index, end);
    } else {
        for (auto it=std::next(begin); it!=std::prev(end);) {
            auto next = std::next(it);

            auto prev = std::prev(it);
            DVector2 diff = *it - *prev;
            double dist2 = dot_product(diff, diff);

            if (dist2 < min_sep2) points.erase(it);

            it = next;
        }
    }
}


void RoadGenerator::push_road(std::list<DVector2>& points, RoadType road, Eigenfield ef) {
    if (points.front() != points.back()) {
        add_candidate_seed(points.front(), flip(ef));
        add_candidate_seed(points.back(), flip(ef));
    }

    insert(points, road, ef);
}

// standard joining candidate algorithm
// std::optional<NodeHandle>
// RoadGenerator::joining_candidate(const double& rad, const double& max_node_sep2, const double& theta_max, 
//     const DVector2& pos, const DVector2& road_direction, const std::unordered_set<NodeHandle>& forbidden) const 
// {
//     std::list<NodeHandle> nearby = 
//         storage_.nearby_points(pos, rad, Major | Minor);
//
//     std::optional<NodeHandle> best_node;
//
//     double min_dist2 = std::numeric_limits<double>::infinity();
//
//     for (const NodeHandle& candidate_handle: nearby) {
//
//         if (forbidden.contains(candidate_handle)) continue;
//
//         DVector2 join_vector = storage_.get_pos(candidate_handle);
//
//         if (dot_product(join_vector, road_direction) < 0) continue; // opposite directions.
//
//         double d2 = dot_product(join_vector, join_vector);
//
//         if (d2 < max_node_sep2)
//             return candidate_handle;
//
//
//         double theta = std::abs(vector_angle(road_direction, join_vector));
//
//         if (theta < theta_max && d2 < min_dist2) {
//             min_dist2 = d2;
//             best_node = candidate_handle;
//         }
//     }
//
//     return best_node;
// }

// void RoadGenerator::connect_roads(RoadType road, Eigenfield dir) {
//     int k = 0;
//     for (Streamline& s : streamlines_[road].get_streamlines(dir)) {
//         if (s.front() == s.back()) continue; // ignore circles
//
//         DVector2 front_pos = nodes_[s.front()].pos;
//         DVector2 back_pos = nodes_[s.back()].pos;
//
//         std::unordered_set<node_id> front_forbidden;
//         std::unordered_set<node_id> back_forbidden;
//
//
//         // first min_streamline_size_ nodes
//         Streamline::iterator last_front_forbidden = std::next(s.begin(), min_streamline_size_-1);
//         for (auto it = s.begin(); it != last_front_forbidden; ++it) {
//             front_forbidden.insert(*it);
//         }
//
//         // DVector2 front_direction = front_pos - nodes_[*std::prev(last_forbidden)].pos;
//         DVector2 front_direction = front_pos - nodes_[*last_front_forbidden].pos;
//
//
//
//         Streamline::iterator last_back_forbidden = std::prev(s.end(), min_streamline_size_);
//         for (auto it = std::prev(s.end()); it != last_back_forbidden; ++it) {
//             back_forbidden.insert(*it);
//         }
//
//
//         DVector2 back_direction = back_pos - nodes_[*last_back_forbidden].pos;
//
//
//         std::optional<node_id> front_join 
//             = joining_candidate(params_.at(road).d_lookahead, params_.at(road).node_sep2, params_.at(road).theta_max,
//                     front_pos, front_direction, front_forbidden);    
//         std::optional<node_id> back_join
//             = joining_candidate(params_.at(road).d_lookahead, params_.at(road).node_sep2, params_.at(road).theta_max,
//                     back_pos, back_direction, back_forbidden);
//
//         if (front_join.has_value()) {
//             // connect(s, s.front(), front_join.value());
//             s.push_front(front_join.value());
//             ++k;
//         }
//
//         if (back_join.has_value()) {
//             // connect(s, s.back(), back_join.value());
//             s.push_back(back_join.value());
//             ++k;
//         }
//
//     }
//
//     std::cout << "Connected " << k << " roads" <<std::endl;
// }


// void RoadGenerator::add_intersections(RoadType road, Eigenfield dir, Streamline& s) {
//     const double& sep = params_.at(road).node_sep;
//
//     for (auto it = s.begin(); it != s.end(); ++it) {
//         std::list<node_id> nearby_opposite_dir = spatial_.nearby_points(nodes_[*it].pos, sep, flip(dir));
//         if (nearby_opposite_dir.empty()) continue;
//
//         // create shared point
//         std::optional<node_id> closest;
//         double d_min = 
//
//         for (const node_id& id : nearby_opposite_dir) {
//             if (dot_product(const TVector2<T> &a, const TVector2<T> &b))
//         }
//     }
// }


RoadGenerator::RoadGenerator(
    std::unique_ptr<NumericalFieldIntegrator>& integrator,
    std::unordered_map<RoadType, GeneratorParameters> parameters,
    Box<double> viewport
) :
    viewport_(viewport),
    integrator_(std::move(integrator)),
    params_(parameters),
    dist_(0.0, 1.0),
    RoadStorage(viewport, kQuadTreeDepth, kQuadTreeLeafCapacity)
{
    road_types_.reserve(parameters.size());

    for (auto& [key, params] : params_) {
        params.d_test = std::min(params.d_test, params.d_sep);
        road_types_.push_back(key);
    }
}


const std::vector<RoadType>&
RoadGenerator::get_road_types() const {
    return road_types_;
}


const std::unordered_map<RoadType, GeneratorParameters>&
RoadGenerator::get_parameters() const {
    return params_;
}




void RoadGenerator::reset(Box<double> new_viewport) {
    viewport_ = new_viewport;
    clear();
}


void RoadGenerator::clear() {
    for (int i=0; i<EigenfieldCount;++i) {
        seeds_[i] = {};
    }

    reset_storage(viewport_);
}


void RoadGenerator::generate() {
    clear();

    std::sort(road_types_.begin(), road_types_.end());

    for (auto r : road_types_) {
        generate_all_roads(r);
    }
}
