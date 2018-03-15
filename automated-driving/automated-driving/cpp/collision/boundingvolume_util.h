#include "boundingvolume.h"
#include <stack>
#include <vector>

namespace collision{

using BoundingVolumeConstPtrPair = std::pair<BoundingVolumeConstPtr, BoundingVolumeConstPtr>;

BoundingVolumeConstPtr buildBoundingVolumeHierarchy(std::vector<CollisionObjectConstPtr> collision_objects_);

class BVHIntersection {

public:
    BVHIntersection(BoundingVolumeConstPtr bvh_a,
                    BoundingVolumeConstPtr bvh_b);

    BoundingVolumeConstPtrPair nextCandidates();
    std::vector<BoundingVolumeConstPtrPair> findAllCandidates();
    bool hasNextCandidates();
    void reset();
private:
    void findNextCandidate();
    void traverseTreeStep();
    void updateNextIntersectionTest();
    BoundingVolumeConstPtr root_bvh_a_;
    BoundingVolumeConstPtr root_bvh_b_;
    std::stack<BoundingVolumeConstPtrPair> pairs_of_bvh_to_check_;
    std::vector<BoundingVolumeConstPtrPair> narrow_phase_candidates_;
    bool new_narrow_phase_candidate_;

};

}
