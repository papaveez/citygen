#include "generator.h"
#include <iostream>

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
RoadGenerator::get_seed(size_t road, Eigenfield ef) {
    seed_queue& candidate_queue = seeds_[ef];

    const GeneratorParameters& param = params_[road];


    DVector2 seed;
    while (!candidate_queue.empty()) {
        DVector2 seed = candidate_queue.front();
        candidate_queue.pop();
        if (!has_nearby_point(seed, param.d_sep, ef)) {
            return seed;
        } 
    }


    for (int count=0; count<param.max_seed_retries; count++) {
        seed = DVector2 {
            dist_(gen_)*viewport_.width()  + viewport_.min.x,
            dist_(gen_)*viewport_.height() + viewport_.min.y
        };


        if (!has_nearby_point(seed, param.d_sep, ef)) {
            return seed;
        }
    }
    return {};
}

DVector2 RoadGenerator::get_eigenvector(const DVector2& x,
    const Eigenfield& ef) const 
{
    Tensor out = field_->sample(x);

    if (ef == Eigenfield::major()) {
        return out.get_major_eigenvector();
    } else {
        return out.get_minor_eigenvector();
    }
}

DVector2 RoadGenerator::integrate_rk4(const DVector2& x,
    const Eigenfield& ef, const double& dl) const 
{
    DVector2 dx = {dl, dl};

    DVector2 k1 = get_eigenvector(x, ef);
    DVector2 k2 = get_eigenvector(x + dx/2.0, ef);
    DVector2 k4 = get_eigenvector(x + dx, ef);

    return k1 + k2*4.0 + k4/6.0;
}



void RoadGenerator::extend_road(
    Integration& res,
    const size_t& road, 
    const Eigenfield& ef 
) const {
    if (res.status != Continue) {
        res.status = Abort;
        return;
    };

    DVector2 delta = integrate_rk4(
        res.integration_front, 
        ef, 
        params_[road].dl
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
    if (has_nearby_point(res.integration_front, params_[road].d_test, ef)) {
        res.status = Terminate;
    }
}


std::list<DVector2>
RoadGenerator::spawn_road(size_t road, DVector2 seed_point, Eigenfield ef) {
    Integration forward  (seed_point, false);
    Integration backward (seed_point, true );

    // circle logic
    bool points_diverged = false;
    bool join = false; // flag to join endpoints (in a circular road)

    int count = 0;

    const auto& max_iter = params_[road].max_integration_iterations;

    while(count<max_iter) {
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

        if (points_diverged && sep2 < params_[road].d_circle2) {
            join = true;
            break;
        } else if (!points_diverged && sep2 > params_[road].d_circle2) {
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

    return result;
}


int RoadGenerator::generate_roads(size_t road_type) {
    Eigenfield ef = Eigenfield::major();

    std::optional<DVector2> seed = get_seed(road_type, ef);
    int k = 0;
    while (seed.has_value()) {
        std::cout << "Seed: " << seed.value() << std::endl;
        std::list<DVector2> streamline = spawn_road(road_type, seed.value(), ef);

        simplify_streamline(road_type, streamline);


        if (streamline.size() >= tangent_samples_) { 
            push_road(streamline, road_type, ef);
            k += 1;

            ef = ef.opposite();
        }
        
        seed = get_seed(road_type, ef);
    };


    // connect_roads(road_type, Major);
    // connect_roads(road_type, Minor);

    return k;
}


void RoadGenerator::simplify_streamline(size_t road, std::list<DVector2>& points) const {
    assert(params_[road].epsilon > 0.0);
    douglas_peucker(params_[road].epsilon, params_[road].node_sep2, points, points.begin(), points.end());
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


void RoadGenerator::push_road(std::list<DVector2>& points, size_t road, Eigenfield ef) {
    if (points.front() != points.back()) {
        add_candidate_seed(points.front(), ef.opposite());
        add_candidate_seed(points.back(), ef.opposite());
    }

    insert(points, road, ef);
}


DVector2 RoadGenerator::tangent(const NodeHandle& handle) const {
    const Road& road = get_road(handle);

    int idx = handle.idx;

    NodeHandle left = handle;
    NodeHandle right = handle;

    int l = tangent_samples_/2;

    left.idx = std::max(static_cast<int>(road.begin), idx-l);
    right.idx = std::min(static_cast<int>(road.end-1), idx+l);

    return get_pos(right)-get_pos(left);
}


std::optional<NodeHandle>
RoadGenerator::joining_candidate(const NodeHandle& handle) const {
    const RoadHandle& road_handle = handle.road_handle;
    const Road& road = get_road(road_handle);

    std::list<NodeHandle> nearby = nearby_points(
        get_pos(handle), 
        params_[road_handle.road_type].d_lookahead,
        Eigenfield::major() | Eigenfield::minor()
    );

    double theta_max = params_[road_handle.road_type].theta_max;
    DVector2 pos = get_pos(handle);
    DVector2 local_dir = tangent(handle);

    bool is_endpoint = (handle.idx == road.end - 1 || handle.idx == road.begin);

    if (handle.idx == road.begin) local_dir = local_dir*-1.0;

    double min_dist2 = std::numeric_limits<double>::infinity();
    std::optional<NodeHandle> best_candidate;

    for (const NodeHandle& candidate : nearby) {
        if (candidate.road_handle == road_handle) continue;

        DVector2 join_vector = get_pos(candidate) - pos;

        if (is_endpoint && dot_product(join_vector, local_dir) < 0)
            continue;

        double dist2 = dot_product(join_vector, join_vector);

        if (dist2 > min_dist2) continue;

        double leave_angle = std::abs(
            vector_angle(local_dir, join_vector)
        );

        if (leave_angle < theta_max) {
            dist2 = min_dist2;
            best_candidate = candidate;
        }
    }

    return best_candidate;
}


std::list<DVector2>
RoadGenerator::joining_streamline(double dl, DVector2 x0, DVector2 x1) const {
    DVector2 diff = (x1 - x0);
    double dist = std::hypot(diff.x, diff.y);

    dl = std::min(dist/tangent_samples_, dl);

    DVector2 inc = diff*dl/dist;

    std::list<DVector2> out = {x0};

    double dl2 = dl*dl;

    while (dot_product(diff, diff) > dl2) {
        out.push_back(out.back() + inc);
        diff = x1 - out.back();
    }

    out.push_back(x1);
    return out;
}


void RoadGenerator::connect_roads(size_t road_type, Eigenfield ef) {
    RoadHandle road_handle {
        0,
        road_type,
        ef
    };

    std::uint32_t count = road_count(road_type, ef);

    double dl = params_[road_type].node_sep;

    for (std::uint32_t idx = 0; idx < count; ++idx) {
        road_handle.idx = idx;
        const Road& road = get_road(road_handle);

        NodeHandle first = { road.begin, road_handle };
        NodeHandle last  = { road.end-1,   road_handle };

        std::optional<NodeHandle> first_join = joining_candidate(first);
        std::optional<NodeHandle> last_join  = joining_candidate(last);

        if (first_join.has_value()) {
            std::list<DVector2> s_join = joining_streamline(
                dl,
                get_pos(first),
                get_pos(first_join.value())
            );

            insert(s_join, road_type, ef, true);
        }
        if (last_join.has_value()) {
            std::list<DVector2> s_join = joining_streamline(
                dl,
                get_pos(last),
                get_pos(last_join.value())
            );

            insert(s_join, road_type, ef, true);
        }
    }
}


RoadGenerator::RoadGenerator(
    TensorField* field,
    size_t road_type_count,
    GeneratorParameters* parameters,
    Box<double> viewport
) :
    viewport_(viewport),
    field_(field),
    dist_(0.0, 1.0),
    RoadStorage(viewport, kQuadTreeDepth, kQuadTreeLeafCapacity, road_type_count),
    params_(parameters)
{

    for (int i=0; i<road_type_count; ++i) {
        GeneratorParameters& p = params_[i];
        p.d_test = std::min(p.d_test, p.d_sep);
    }
}


size_t RoadGenerator::road_type_count() const {
    return road_type_count_;
}

void RoadGenerator::reset(Box<double> new_viewport) {
    viewport_ = new_viewport;
    clear();
}


void RoadGenerator::clear() {
    for (int i=0; i<Eigenfield::count;++i) {
        seeds_[i] = {};
    }

    reset_storage(viewport_);
}


void RoadGenerator::generate() {
    clear();


    for (int i=0;i<road_type_count_;++i) {
        generate_roads(i);
    }
}
