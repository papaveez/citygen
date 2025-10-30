#include "road_storage.h"


Eigenfield flip(Eigenfield ef) {
    return Eigenfield(ef.value == Major ? Minor : Major);
}


const Road& RoadStorage::get_road(const RoadHandle& h) const {
    return roads_[h.road_type][h.eigenfield][h.idx];
}


std::array<std::pair<ef_mask, std::list<NodeHandle>>, 4> 
RoadStorage::partition(const Box<double>& bbox, std::list<NodeHandle>& s) {
    DVector2 mid = middle(bbox.min, bbox.max);

    auto quadrant_id = [&mid, this](const NodeHandle& h) {
        const DVector2& pos = get_pos(h);
        return (pos.x > mid.x) + ((pos.y > mid.y)<<1);
    };



    std::array<std::pair<ef_mask, std::list<NodeHandle>>, 4> out;

    for (auto it = s.begin(); it != s.end();) {
        auto curr = it++;
        int q = quadrant_id(*curr);

        out[q].first |= get_eigenfields(*curr);
        out[q].second.splice(out[q].second.end(), s, curr);
    }

    return out;
}


bool RoadStorage::is_leaf(const qnode_id& id) const {
    const QuadNode& root_node = qnodes_[id];

    for (int i=0; i<4; ++i) {
        if (root_node.children[i] != NullQNode) return false;
    }

    return true;
}


void RoadStorage::subdivide(const qnode_id& head_ptr) {
    const Box<double> bbox = qnodes_[head_ptr].bbox;
    auto parts = partition(bbox, qnodes_[head_ptr].data);

    std::array<Box<double>, 4> sub_bbox = bbox.quadrants();

    for (int i=0;i<4; ++i) {
        auto& [eigenfields, sublist] = parts[i];
        if (sublist.empty()) continue;

        qnode_id child_ptr = qnodes_.size();
        qnodes_.emplace_back(sub_bbox[i], eigenfields);
        qnodes_[child_ptr].data = std::move(sublist);

        qnodes_[head_ptr].children[i] = child_ptr;
    }
}


void RoadStorage::append_leaf_data(const qnode_id& leaf_ptr,
    const ef_mask& eigenfields, std::list<NodeHandle>& data) 
{
    QuadNode& leaf = qnodes_[leaf_ptr];

    leaf.eigenfields |= eigenfields;
    leaf.data.splice(leaf.data.end(), data);
}


void RoadStorage::insert_rec(int depth, const qnode_id& head_ptr, 
    const ef_mask& eigenfields, std::list<NodeHandle>& list)
{

    // base cases
    if (depth >= max_depth_) {
        // 1: Max Depth Exceeded 
        append_leaf_data(head_ptr, eigenfields, list);
        return;
    } 
    else if (is_leaf(head_ptr)) {
        if (qnodes_[head_ptr].data.size() + list.size() <= leaf_capacity_) {
            // 2: Leaf has space
            append_leaf_data(head_ptr, eigenfields, list);
            return;
        }

        // subdivide(head_ptr);
        list.splice(list.end(), qnodes_[head_ptr].data);
    }

    qnodes_[head_ptr].eigenfields |= eigenfields;

    Box<double> bbox = qnodes_[head_ptr].bbox;
    auto parts = partition(bbox, list);

    ++depth;

    std::array<Box<double>, 4> quadrants = bbox.quadrants();

    for (int q=0;q<4;++q) {
        auto& [sub_dirs, sublist] = parts[q];

        if (sublist.empty()) continue;
        qnode_id child_ptr = qnodes_[head_ptr].children[q];

        if (child_ptr == NullQNode) {
            child_ptr = qnodes_.size();
            qnodes_.emplace_back(quadrants[q], sub_dirs);
            qnodes_[head_ptr].children[q] = child_ptr;
        }

        insert_rec(
            depth,
            child_ptr,
            sub_dirs,
            sublist
        );
    }
}


bool
RoadStorage::in_circle_rec(const qnode_id& head_ptr,
        CircleQuery& query) const 
{
    const QuadNode& head = qnodes_[head_ptr];

    if ( !(head.eigenfields & query.eigenfields) 
        || (query.outer_bbox & head.bbox).is_empty()) 
    {
        return false;
    }

    // if query.inner_bbox ⊆ qnode.bbox
    if ((head.bbox | query.inner_bbox) == query.inner_bbox) {
        return in_bbox_rec(head_ptr, query);
    }

    bool flag = false;


    if (is_leaf(head_ptr)) {
        for (const NodeHandle& handle : head.data) {
            if (!(get_eigenfields(handle) & query.eigenfields))
                continue;

            DVector2 sep = query.centre - get_pos(handle);
            if (dot_product(sep, sep) > query.radius2) continue;

            if (query.gather) {
                query.harvest.push_back(handle);
                flag = true;
            } else {
                return true;
            }
        }

        return flag;
    }

    for (int q=0; q<4; ++q) {
        const qnode_id& child_ptr = head.children[q];

        if (child_ptr == NullQNode) continue;
        

        if (in_circle_rec(child_ptr, query)) {
            if (query.gather) {
                flag = true;
            } else {
                return true;
            }
        }
    }

    return flag;
}


bool
RoadStorage::in_bbox_rec(const qnode_id& head_ptr, BBoxQuery& query) const {
    const QuadNode& head = qnodes_[head_ptr];

    // TODO: fix
    //
    if ( ( query.inner_bbox & head.bbox ).is_empty() 
        || !(head.eigenfields & query.eigenfields))
        return false;

    if ((query.inner_bbox | head.bbox) == query.inner_bbox) {
        if (query.gather && !head.data.empty()) {
            for (auto& hd : head.data) {
                if (!(get_eigenfields(hd) & query.eigenfields)) continue;

                query.harvest.push_back(hd);
            }

            return !is_leaf(head_ptr) || !head.data.empty();
        } else if (!query.gather) {
            return !is_leaf(head_ptr) || !head.data.empty();
        }
    }


    bool flag = false;

    if (is_leaf(head_ptr)) {
        for (const NodeHandle& hd : head.data)  {
            if (query.inner_bbox.contains(get_pos(hd))
                && (get_eigenfields(hd) & query.eigenfields))
            {
                if (query.gather) {
                    query.harvest.push_back(hd);
                    flag = true;
                } else {
                    return true;
                }
            }
        }

        return flag;
    }


    for (int i=0; i<4; ++i) {
        qnode_id child_ptr = head.children[i];
        if (child_ptr == NullQNode) continue;

        if (in_bbox_rec(child_ptr, query)) {
            if (query.gather) {
                flag = true;
            } else {
                return true;
            }
        }
    }

    return flag;
}


RoadStorage::RoadStorage(
    Box<double> viewport,
    int depth,
    int leaf_capacity
) :
    viewport_(viewport),
    max_depth_(depth),
    leaf_capacity_(leaf_capacity) 
{
    reset_storage(viewport);
}


const DVector2& RoadStorage::get_pos(const NodeHandle& h) const {
    return nodes_[h.idx];
}


ef_mask RoadStorage::get_eigenfields(const NodeHandle& h) const {
    return h.road_handle.eigenfield.mask();
}


void RoadStorage::reset_storage(Box<double> new_viewport) {
    viewport_ = new_viewport;
    root_ = 0;
    qnodes_.clear();
    qnodes_.emplace_back(new_viewport, 0);

    nodes_.clear();
    fnodes_.clear();

    for (int i=0; i<RoadTypeCount; ++i) {
        for (int j=0; j < EigenfieldCount; ++j) {
            roads_[i][j].clear();
        }
    }
}



void RoadStorage::insert(const std::list<DVector2>& points,
    RoadType road_type, Eigenfield eigenfield) {
    if (points.size() == 0) return;

    std::list<NodeHandle> node_handles;

    Road new_road = {
        static_cast<std::uint32_t>(nodes_.size()),
        static_cast<std::uint32_t>(nodes_.size() + points.size())
    };


    RoadHandle new_road_handle = {
        static_cast<std::uint32_t>(roads_[road_type][eigenfield].size()),
        road_type,
        eigenfield
    };


    std::uint32_t idx = nodes_.size();

    for (const auto& pt : points) {
        assert(idx != -1);

        nodes_.push_back(pt);
        fnodes_.push_back(pt);
        node_handles.push_back({
            idx,
            new_road_handle
        });

        ++idx;
    }


    roads_[road_type][eigenfield].push_back(new_road);
    insert_rec(0, root_, eigenfield.mask(), node_handles);
}


std::pair<size_t, const Vector2*>
RoadStorage::get_road_points(const RoadHandle& road_handle) const {
    const Road& road = get_road(road_handle);

    return {
        road.end - road.begin,
        fnodes_.data() + road.begin
    };
}


std::uint32_t RoadStorage::road_count(RoadType road_type, Eigenfield eigenfield) const {
    return roads_[road_type][eigenfield].size();
}


bool 
RoadStorage::has_nearby_point(DVector2 centre, double radius, ef_mask eigenfields) const {
    CircleQuery query(eigenfields, centre, radius, false);
    return in_circle_rec(root_, query);
}

std::list<NodeHandle>
RoadStorage::nearby_points(DVector2 centre, double radius, ef_mask eigenfields) const {
    CircleQuery query(eigenfields, centre, radius, true);
    in_circle_rec(root_, query);
    return query.harvest;
}
