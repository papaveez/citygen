#ifndef ROAD_STORAGE_H 
#define ROAD_STORAGE_H

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <list>
#include <vector>

#include "integrator.h"
#include "../types.h"
#include "../const.h"


using qnode_id = std::uint32_t;                
constexpr qnode_id NullQNode = -1;


enum RoadType : size_t {
    Main,
    HighStreet,
    SideStreet,
    RoadTypeCount
};

using ef_mask = char;

struct Eigenfield {
    enum Direction value;

    constexpr Eigenfield(Direction v) : value(v) {}

    constexpr operator size_t() const {return value;}
    constexpr operator ef_mask() const {return 1<<value;}
    constexpr operator Direction() const {return value;}
    constexpr ef_mask mask() const {return 1<<value;}

    constexpr bool operator == (const Eigenfield& other) const {return value == other.value;}
    constexpr bool operator == (const Direction& other) const {return value == other;}

    constexpr ef_mask operator | (const Eigenfield& other) const {
        return static_cast<ef_mask>(other) | static_cast<ef_mask>(*this);
    }
};

Eigenfield flip(Eigenfield ef);


struct RoadHandle {
    std::uint32_t idx;
    RoadType road_type;
    Eigenfield eigenfield;

    bool operator==(const RoadHandle& other) const {
        return idx==other.idx 
            && road_type==other.road_type 
            && eigenfield == other.eigenfield;
    }
};


struct Road {
    std::uint32_t begin;
    std::uint32_t end;
    bool is_joining_road;
};


struct NodeHandle {
    std::uint32_t idx;
    RoadHandle road_handle;
};

struct QuadNode {
    Box<double> bbox;

    std::list<NodeHandle> data;

    qnode_id children[4] = {NullQNode, NullQNode, NullQNode, NullQNode};

    ef_mask eigenfields; // bit mask saying what direction (Major/Minor) children have
    
    QuadNode(Box<double> bounding_box, ef_mask eigenfields) :
        bbox(bounding_box),
        eigenfields(eigenfields)
    {}
};



class RoadStorage {
private:
    struct BBoxQuery {
        ef_mask eigenfields;
        bool gather;
        Box<double> inner_bbox;
        std::list<NodeHandle> harvest;
    };

    struct CircleQuery : BBoxQuery {
        DVector2 centre;
        double radius;
        double radius2;
        Box<double> outer_bbox;
        CircleQuery(ef_mask eigenfields, DVector2 c, double r, bool g) : 
            BBoxQuery({eigenfields, g}),
            centre(c),
            radius(r) 
        {
            radius2 = radius*radius;

            DVector2 circumscribed_diag = {radius, radius};
            DVector2 inscribed_diag = circumscribed_diag/M_SQRT2;

            outer_bbox = Box (
                centre - circumscribed_diag,
                centre + circumscribed_diag
            );


            inner_bbox = Box(
                centre - inscribed_diag,
                centre + inscribed_diag 
            );
        }
    };

    // node storage
    std::vector<DVector2> nodes_;
    std::vector<Vector2> fnodes_; // quick conversion to float for rendering
    std::array<std::array<std::vector<Road>, EigenfieldCount>, RoadTypeCount> 
        roads_;

    // quadtree
    Box<double> viewport_;

#ifdef STORAGE_TEST
public:
#endif
    qnode_id root_;
    std::vector<QuadNode> qnodes_;

    int max_depth_;
    int leaf_capacity_;


    std::array<std::pair<ef_mask, std::list<NodeHandle>>, 4> 
        partition(const Box<double>& bbox, std::list<NodeHandle>& s);

    bool is_leaf(const qnode_id& id) const;

    void append_leaf_data(
        const qnode_id& leaf_ptr,
        const ef_mask& eigenfields,
        std::list<NodeHandle>& data
    );

    void insert_rec(
        const int& depth, 
        const qnode_id& head_ptr,
        const ef_mask& dirs,
        std::list<NodeHandle>& list
    );

    bool in_circle_rec(
        const qnode_id& head_ptr,
        CircleQuery& query
    ) const;

    bool in_bbox_rec(
        const qnode_id& head_ptr,
        BBoxQuery& query
    ) const;

    bool gather_data_rec(
        const qnode_id& head_ptr,
        BBoxQuery& query
    ) const;

protected:
    RoadStorage(
        Box<double> viewport,
        int depth,
        int leaf_capacity
    );

#ifdef STORAGE_TEST
public:
#endif
    const DVector2& get_pos(const NodeHandle& h) const;
    ef_mask get_eigenfields(const NodeHandle& h) const;

    const Road& get_road(const RoadHandle& h) const;
    const Road& get_road(const NodeHandle& h) const;

    void reset_storage(Box<double> new_viewport);

    void insert(
        const std::list<DVector2>& points,
        RoadType road_type,
        Eigenfield eigenfield,
        bool is_join = false
    );
    
    bool has_nearby_point(
        DVector2 centre,
        double radius,
        ef_mask eigenfields
    ) const;

    std::list<NodeHandle> nearby_points(
        DVector2 centre,
        double radius,
        ef_mask eigenfields
    ) const;

public:
    std::pair<size_t, const Vector2*> get_road_points(const RoadHandle& road_handle) const;
    std::uint32_t road_count(RoadType road_type, Eigenfield eigenfield) const;


    bool is_connective_road(const RoadHandle& rh) const;
};

#endif

