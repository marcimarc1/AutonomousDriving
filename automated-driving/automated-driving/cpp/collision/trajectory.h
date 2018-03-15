#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <iostream>

#include "timevariant_collisionobject.h"
#include "boundingvolume.h"

namespace collision {

class Trajectory : public TimeVariantCollisionObject {

public:

    Trajectory(int time_start_idx, std::vector<Eigen::Vector2d> positions, std::vector<double> orientations,
               double length, double width, double max_interp_distance = 1.0);

    virtual ~Trajectory() {}

    //! Print all parameters of the trajectory
    void print(std::ostringstream &stream) const;
    BoundingVolumeConstPtr getBoundingVolume() const;

private:
    std::vector<Eigen::Vector2d> positions_;
    std::vector<double> orientations_;

private:
    using TimeVariantCollisionObject::time_start_idx_;
    using TimeVariantCollisionObject::time_end_idx_;
    using TimeVariantCollisionObject::collision_object_at_time_;

};

typedef std::shared_ptr<Trajectory> TrajectoryPtr;
typedef std::shared_ptr<const Trajectory> TrajectoryConstPtr;

}

#endif
