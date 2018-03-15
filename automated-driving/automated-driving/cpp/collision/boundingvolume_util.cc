#include "boundingvolume_util.h"
#include "collision_object.h"
#include "primitive_collision_tests.h"

#include <assert.h>
#include <limits.h>

namespace collision{
    
BVHIntersection::BVHIntersection(BoundingVolumeConstPtr bvh_a,
                                 BoundingVolumeConstPtr bvh_b) {
    root_bvh_a_ = bvh_a;
    root_bvh_b_ = bvh_b;
    pairs_of_bvh_to_check_.push(BoundingVolumeConstPtrPair(root_bvh_a_,
                                                           root_bvh_b_));
    reset();
}

void BVHIntersection::reset() {
    new_narrow_phase_candidate_ = false;
    narrow_phase_candidates_.clear();
    while (!pairs_of_bvh_to_check_.empty()) {
        pairs_of_bvh_to_check_.pop();
    }
    pairs_of_bvh_to_check_.push(BoundingVolumeConstPtrPair(root_bvh_a_,
                                                           root_bvh_b_));
    findNextCandidate();
}

bool BVHIntersection::hasNextCandidates() {
    return new_narrow_phase_candidate_;
}

void BVHIntersection::findNextCandidate() {
    int n_candidates = narrow_phase_candidates_.size();
    while (!pairs_of_bvh_to_check_.empty()) {
        traverseTreeStep();
        if (narrow_phase_candidates_.size() > n_candidates) {
            new_narrow_phase_candidate_ = true;
            return;
        }
    }
}

BoundingVolumeConstPtrPair BVHIntersection::nextCandidates() {
    if (new_narrow_phase_candidate_) {
        new_narrow_phase_candidate_ = false;
        auto ret = narrow_phase_candidates_.back();
        findNextCandidate();
        return ret;
    } else {
        return BoundingVolumeConstPtrPair(nullptr, nullptr);
    }
    
}

inline static bool overlapBVH(BoundingVolume bv_first,
                                  BoundingVolume bv_second) {
    if (bv_first.t()[1] < bv_second.t()[0] || bv_first.t()[0] > bv_second.t()[1]) return false;
    if (bv_first.x()[1] < bv_second.x()[0] || bv_first.x()[0] > bv_second.x()[1]) return false;
    if (bv_first.y()[1] < bv_second.y()[0] || bv_first.y()[0] > bv_second.y()[1]) return false;
    return true;
}
    
std::vector<BoundingVolumeConstPtrPair> BVHIntersection::findAllCandidates() {
    reset();
    while (!pairs_of_bvh_to_check_.empty()) {
        traverseTreeStep();
    }
    return narrow_phase_candidates_;
}

void BVHIntersection::traverseTreeStep() {
    assert(~pairs_of_bvh_to_check_.empty());
    auto pair_to_check = pairs_of_bvh_to_check_.top();
    pairs_of_bvh_to_check_.pop();
    if (overlapBVH(*pair_to_check.first, *pair_to_check.second)) {
        if (pair_to_check.first->isLeaf() && pair_to_check.second->isLeaf()) {
            narrow_phase_candidates_.push_back(pair_to_check);
        } else if (!pair_to_check.first->isLeaf()) {
            for (auto child : pair_to_check.first->getChildrenBoundingVolumes()) {
                pairs_of_bvh_to_check_.push(BoundingVolumeConstPtrPair(child, pair_to_check.second));
            }
        } else {
            for (auto child : pair_to_check.second->getChildrenBoundingVolumes()) {
                pairs_of_bvh_to_check_.push(BoundingVolumeConstPtrPair(pair_to_check.first, child));
            }
        }
    }
    
}

BoundingVolumeConstPtr buildBoundingVolumeHierarchy(std::vector<CollisionObjectConstPtr> collision_objects_)
{
    double BVH_TIME_FACTOR = 1.0;

    struct bv_match
    {
        BoundingVolumeConstPtr bv_first;
        BoundingVolumeConstPtr bv_second;
        double volume;
        bool operator()(bv_match bvm1, bv_match bvm2) {
            return (bvm1.volume > bvm2.volume);  // operator for storting, ">" for negative sorting
            } 
    } comperator;  // instance for vector sorting needed

    // initialize the bvh pointer
    BoundingVolumeConstPtr bounding_volume_hierarchy_;

    //  build up the BVs for all leaf nodes
    std::vector<BoundingVolumeConstPtr> bvs;  // statt einem reinen Vektor kann hier auch eine eigene Klasse verwendet werden, die z.B. auch eine bestmatch() Funktion hat
    std::array<int, 2> t_borders = {INT_MAX, INT_MIN};
    for (auto &co : collision_objects_)
    {
        BoundingVolumeConstPtr temp = co->getBoundingVolume();
        bvs.push_back(temp);

        std::array<int, 2> t_temp = temp->t();
        // non-static element
        if (t_temp[0] >= 0)
        {
            if (t_temp[0] < t_borders[0])
            {
                t_borders[0] = t_temp[0];
            }
            if (t_temp[1] > t_borders[1])
            {
                t_borders[1] = t_temp[1];
            }
        }
    }
    if (t_borders[0] == INT_MAX && t_borders[1] == INT_MIN)
    {
        t_borders = {0, 1};
    } // only static objects --> Static factor becomes 1

    //  build the priority queue for the matching nodes
    std::vector<bv_match> matches; //statt einem Vektor kann hier auch eine std::priority_queue verwendet werden
    for (BoundingVolumeConstPtr &bv1 : bvs)
    {
        BoundingVolumeConstPtr best_match;
        double best_volume = INFINITY;

        for (BoundingVolumeConstPtr &bv2 : bvs)
        {
            double x_temp = std::max(bv1->x()[1], bv2->x()[1]) -
                            std::min(bv1->x()[0], bv2->x()[0]);
            double y_temp = std::max(bv1->y()[1], bv2->y()[1]) -
                            std::min(bv1->y()[0], bv2->y()[0]);
            double t_temp = t_borders[1] - t_borders[0];
            if (bv1->t()[0] > -1 && bv2->t()[0] > -1)
            { // both are non-static objects
                t_temp = std::max(bv1->t()[1], bv2->t()[1]) - 
                         std::min(bv1->t()[0], bv2->t()[0]);
            }
            double volume_temp = x_temp * y_temp * t_temp * BVH_TIME_FACTOR;

            if (volume_temp < best_volume && bv1 != bv2)
            {
                best_volume = volume_temp;
                best_match = bv2;
            }
        }
        matches.push_back({bv1, best_match, best_volume});
    }
    // sort using overloaded operator
    std::sort(matches.begin(), matches.end(), comperator);

    // iteratively pair and drop bvs until there is only one bounding volume (i.e. only one match)
    while (matches.size() > 1)
    {
        bv_match match_pair = matches.back();
        matches.pop_back();

        //  check if node is already paired (not in bvs any more)
        bool in = false;
        for (BoundingVolumeConstPtr &bv : bvs)
        {
            if (match_pair.bv_first == bv)
            {
                in = true;
                continue;
            }
        }
        if (in == false)
        {
            continue;
        } //continue with the next match

        // recompute the best matching bv for bv_first
        BoundingVolumeConstPtr re_best_match;
        double re_best_volume = INFINITY;

        for (BoundingVolumeConstPtr &bv_partner : bvs)
        {
            double re_x_temp = std::max(match_pair.bv_first->x()[1],
                                        bv_partner->x()[1]) - 
                               std::min(match_pair.bv_first->x()[0],
                                        bv_partner->x()[0]);
            double re_y_temp = std::max(match_pair.bv_first->y()[1], bv_partner->y()[1]) -
                               std::min(match_pair.bv_first->y()[0], bv_partner->y()[0]);
            double re_t_temp = t_borders[1] - t_borders[0];
            if (match_pair.bv_first->t()[0] > -1 && bv_partner->t()[0] > -1)
            { // both are non-static objects
                re_t_temp = std::max(match_pair.bv_first->t()[1], bv_partner->t()[1]) - 
                            std::min(match_pair.bv_first->t()[0], bv_partner->t()[0]);
            }
            double re_volume_temp = re_x_temp * re_y_temp * re_t_temp * BVH_TIME_FACTOR;

            if (re_volume_temp < re_best_volume && match_pair.bv_first != bv_partner)
            {
                re_best_volume = re_volume_temp;
                re_best_match = bv_partner;
            }
        }
        // Check, if still the best match
        if (match_pair.bv_second == re_best_match)
        {

            // pair the best match
            std::vector<BoundingVolumeConstPtr> pair_bvs;
            pair_bvs.push_back(match_pair.bv_first);
            pair_bvs.push_back(match_pair.bv_second);
            BoundingVolume *pair_bv = new BoundingVolume(pair_bvs);

            // delete paired nodes
            std::vector<int> bv_idx;
            for (int i = bvs.size() - 1; i >= 0; i--)
            { //invertion for safe deletion of the correct elements
                if (match_pair.bv_first == bvs[i] || match_pair.bv_second == bvs[i])
                {
                    bv_idx.push_back(i);
                }
            }

            for (int &element : bv_idx)
            {
                bvs.erase(bvs.begin() + element);
            }

            // add new BV to the tree or return the upmost Bounding Volume
            std::shared_ptr<BoundingVolume> pair_bv_ptr = std::shared_ptr<BoundingVolume>(pair_bv);
            if (bvs.size() == 0)
            {
                // set and return the upmost bounding volume
                bounding_volume_hierarchy_ = pair_bv_ptr;
                return pair_bv_ptr; // break condition for last Bounding Volume --> fully built up tree
            }
            else
            {
                bvs.push_back(pair_bv_ptr);
            }

            // Compute the best pairing node for the new node; insert it into queue
            BoundingVolumeConstPtr new_best_match;
            double new_best_volume = INFINITY;

            for (BoundingVolumeConstPtr &new_bv_partner : bvs)
            {
                double new_x_temp = std::max(match_pair.bv_first->x()[1], new_bv_partner->x()[1]) - 
                                    std::min(match_pair.bv_first->x()[0], new_bv_partner->x()[0]);
                double new_y_temp = std::max(match_pair.bv_first->y()[1], new_bv_partner->y()[1]) -
                                    std::min(match_pair.bv_first->y()[0], new_bv_partner->y()[0]);
                double new_t_temp = t_borders[1] - t_borders[0];
                if (match_pair.bv_first->t()[0] > -1 && new_bv_partner->t()[0] > -1)
                { // both are non-static objects
                    new_t_temp = std::max(match_pair.bv_first->t()[1], new_bv_partner->t()[1]) - 
                                 std::min(match_pair.bv_first->t()[0], new_bv_partner->t()[0]);
                }
                double new_volume_temp = new_x_temp * new_y_temp * new_t_temp * BVH_TIME_FACTOR;

                if (new_volume_temp < new_best_volume && match_pair.bv_first != new_bv_partner)
                {
                    new_best_volume = new_volume_temp;
                    new_best_match = new_bv_partner;
                }
            }
            // add pair for the new node
            matches.push_back({pair_bv_ptr, new_best_match, new_best_volume});
        }
        else
        { //add new best match for that node
            matches.push_back({match_pair.bv_first, re_best_match, re_best_volume});
        }

        // sort list of matches using overloaded operator
        std::sort(matches.begin(), matches.end(), comperator);
    }
    return bounding_volume_hierarchy_;
}

}
