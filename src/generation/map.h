#ifndef MAP_H
#define MAP_H

#include <algorithm>
#include <cstddef>
#include <list>

#include "../util.h"



// QuadTree : Look Up Spatial -> returns some handle
template <class V>
struct QuadNode {
    Box bbox;
    std::list<std::pair<Vector2, V>> data;
    std::optional<size_t> children[4];
    QuadNode(Box bounding_box) : bbox(bounding_box) {
        for (int i=0;i<4;++i) children[i] = std::nullopt;
    }


    // TODO: check move semantics
    static QuadNode with_data(Box bounding_box, std::list<std::pair<Vector2, V>>&& data) {
        auto q = QuadNode(bounding_box);
        q.data = std::move(data);
        return q;
    }

    bool is_leaf() const {
        for (int i=0;i<4;++i) {
            if (children[i].has_value()) return false;
        }

        return true;
    }


    void add(Vector2 pos, const V& value) {
        data.push_back(std::make_pair(pos, value));
    }

    void append_data(std::list<std::pair<Vector2, V>>& fresh) {
        data.splice(data.end(), fresh, fresh.begin(), fresh.end());
    }
};

template <class V>
class QuadTree {
private:
    const int kMaxDepth = 10;
    const int kLeafCapacity = 10;
    const Box viewport_;
    std::vector<QuadNode<V>> tree;

    std::array<std::list<std::pair<Vector2, V>>, 4> 
    partition(const Box& bbox, std::list<std::pair<Vector2, V>> points) {
        Vector2 mid = middle(bbox.min, bbox.max);

        auto which_quadrant = [&mid, this](const Vector2& pos) {
            // 0 -> TopLeft
            // 1 -> TopRight
            // 2 -> BottomLeft
            // 3 -> BottomRight
            bool is_right = pos.x < mid.x;
            bool is_bottom = pos.y < mid.y;
            return (is_bottom << 1) + is_right;
        };

        std::array<std::list<std::pair<Vector2, V>>, 4> partition;

        for (auto it = points.begin(); it != points.end();) {
            auto curr = it++;
            const auto& [pos, value] = *curr;
            std::list<std::pair<Vector2, V>>& part = partition[which_quadrant(pos)];

            part.splice(part.end(), points, curr);
        }

        return partition;
    }

    void subdivide_leaf(size_t leaf_id) {
        QuadNode<V>& node = tree[leaf_id];
        assert(node.is_leaf()); // INVARIANT: only subdividing leaves

        auto partitioned = partition(node.bbox, node.data);

        for (int i=0; i<4; ++i) {
            if (!partitioned[i].size()) continue;
            Box child_bbox = node.bbox.get_quadrant(i);
            QuadNode<V> child = QuadNode<V>::with_data(child_bbox, partitioned[i]);

            size_t child_id = tree.size();
            tree.push_back(child);

            node.children[i] = child_id;
        }
    }
    

    void insert_many_rec(int depth, size_t head_ptr, std::list<std::pair<Vector2, V>>& data) {
        if (depth >= kMaxDepth) {
            tree[head_ptr].append_data(data);
            return;
        } else if (tree[head_ptr].is_leaf()) {
            int num_holding = tree[head_ptr].data.size();
            if (num_holding + data.size() <= kLeafCapacity) {
                tree[head_ptr].append_data(data);
                return;
            }

            subdivide_leaf(head_ptr);
        }

        const QuadNode<V>& head = tree[head_ptr];

        // partition nodes into subleaves
        std::array<std::list<Vector2, V>, 4> partitioned = partition(head.bbox, data);

        for (int i=0; i<4; ++i) {
            if (partitioned[i].empty()) continue;

            if (!head.children[i]) {
                Box sub_bbox = head.bbox.get_quadrant(i);
                size_t new_node_id = tree.size();
                tree.push_back(QuadNode<V>(sub_bbox));
                head.children[i] = new_node_id;
            }

            insert_many_rec(depth+1, head.children[i].value(), partitioned[i]);
        }
    }

QuadTree(Box viewport) : viewport_(viewport) {
        tree.push_back(QuadNode<V>(viewport));
    }
};

#endif
