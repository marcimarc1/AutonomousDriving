#ifndef BOUNDINGVOLUME_H_
#define BOUNDINGVOLUME_H_

#include <iostream>
#include <vector>
#include <array>
#include "collision_object.h"

namespace collision {

//class BoundingVolume : public CollisionObject {
class BoundingVolume{

public:

    BoundingVolume(std::array<double,2> x, std::array<double,2> y, std::array<int,2> t, CollisionObjectConstPtr = NULL);
    BoundingVolume(std::vector<BoundingVolumeConstPtr>);

    virtual ~BoundingVolume() {};

    //! Dispatcher functions
    // bool collide(const CollisionObject& c) const;
    // bool collide(const Shape& shape) const;
    // bool collide(const Point& point) const;
    // bool collide(const RectangleAABB& aabb) const;
    // bool collide(const RectangleOBB& obb) const;
    // bool collide(const Sphere& sphere) const;
    // bool collide(const Triangle& triangle) const;
    // bool collide(const TimeVariantCollisionObject& tvco) const;
    // bool collide(const ShapeGroup& sg) const;
    // bool collide(const BoundingVolume& bv) const;

    // //! Print all parameters of the bounding volume
    // void print(std::ostringstream &stream) const;
    // virtual CollisionObjectConstPtr timeSlice(int time_idx, CollisionObjectConstPtr shared_ptr_this) const;
    BoundingVolumeConstPtr getBoundingVolume() const;
    bool isLeaf() const;

    //! Get member variables
    std::vector<BoundingVolumeConstPtr> getChildrenBoundingVolumes() const;
    std::vector<CollisionObjectConstPtr> getChildrenCollisionObjects() const;
    std::array<double,2> x() const;
    std::array<double,2> y() const;
    std::array<int,2> t() const;

private:
    std::array<double,2> x_;
    std::array<double,2> y_;
    std::array<int,2> t_;
    std::vector<BoundingVolumeConstPtr> children_BV_;
    std::vector<CollisionObjectConstPtr> children_CO_;
};

}

#endif
