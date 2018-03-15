#include "boundingvolume.h"
#include "primitive_collision_tests.h"
#include <limits.h>

#include <assert.h>

namespace collision {

BoundingVolume::BoundingVolume(std::array<double,2> x, std::array<double,2> y, std::array<int,2> t, CollisionObjectConstPtr co_ptr) {
    x_ = x;
    y_ = y;
    t_ = t;
    if (co_ptr != NULL) {
        children_CO_.push_back(co_ptr);
    }
}

bool BoundingVolume::isLeaf() const {
    return children_BV_.empty();
}


BoundingVolume::BoundingVolume(std::vector<BoundingVolumeConstPtr> bv_ptr_vector) {
    x_ = {INFINITY,-INFINITY};
    y_ = {INFINITY,-INFINITY};
    t_ = {INT_MAX, INT_MIN}; //maximum int-values


    for (auto &bv : bv_ptr_vector) {
        std::array<double,2> x_temp = bv->x();
        std::array<double,2> y_temp = bv->y();
        std::array<int,2> t_temp = bv->t();
        if (x_temp[0] < x_[0]) { x_[0] = x_temp[0]; }
        if (x_temp[1] > x_[1]) { x_[1] = x_temp[1]; }
        if (y_temp[0] < y_[0]) { y_[0] = y_temp[0]; }
        if (y_temp[1] > y_[1]) { y_[1] = y_temp[1]; }

        // at least one static element
        if (t_temp[0] < 0) { t_ = {-1,-1}; }
        // for only non-static objects
        if (t_[0] != -1 && t_temp[0] >= 0) {
            if (t_temp[0] < t_[0]) { t_[0] = t_temp[0]; }
            if (t_temp[1] > t_[1]) { t_[1] = t_temp[1]; }
        }
        children_BV_.push_back(bv);
    }
}
/*
//! Dispatcher functions
bool BoundingVolume::collide(const CollisionObject& c) const {
  return c.collide(*this);
}

bool BoundingVolume::collide(const Shape& shape) const {
    return shape.collide(*this);
}

bool BoundingVolume::collide(const Point& point) const {
    return primitive_tests::collisionDetection(point, *this);
}

bool BoundingVolume::collide(const RectangleAABB& aabb) const {
    return primitive_tests::collisionDetection(*this, aabb);
}

bool BoundingVolume::collide(const RectangleOBB& obb) const {
    return primitive_tests::collisionDetection(*this, obb);
}

bool BoundingVolume::collide(const Sphere& sphere) const {
    return primitive_tests::collisionDetection(*this, sphere);
}

bool BoundingVolume::collide(const Triangle& triangle) const {
    return primitive_tests::collisionDetection(*this, triangle);
}

bool BoundingVolume::collide(const BoundingVolume& bv) const {
  return primitive_tests::collisionDetection(*this, bv);
}

bool BoundingVolume::collide(const TimeVariantCollisionObject& tvco) const {
    return tvco.collide(*this);
}

bool BoundingVolume::collide(const ShapeGroup& sg) const {
  return sg.collide(*this);
}

void BoundingVolume::print(std::ostringstream &stream) const {
    stream << "Bounding Volume" << std::endl
    << "X: [" << x_[0] << ", " << x_[1] << "]" << std::endl
    << "Y: [" << y_[0] << ", " << y_[1] << "]" << std::endl
    << "t: [" << t_[0] << ", " << t_[1] << "]" << std::endl
    << children_CO_.size() << " COs, " << children_BV_.size() << " BVs" << std::endl;
}

CollisionObjectConstPtr BoundingVolume::timeSlice(int time_idx, CollisionObjectConstPtr shared_ptr_this) const  {
	return shared_ptr_this;
}
*/
BoundingVolumeConstPtr BoundingVolume::getBoundingVolume() const {
    return std::shared_ptr<const BoundingVolume>(this);
}


std::vector<BoundingVolumeConstPtr> BoundingVolume::getChildrenBoundingVolumes() const {
    return children_BV_;
}
std::vector<CollisionObjectConstPtr> BoundingVolume::getChildrenCollisionObjects() const {
    return children_CO_;
}
std::array<double,2> BoundingVolume::x() const{
    return x_;
}
std::array<double,2> BoundingVolume::y() const{
    return y_;
}
std::array<int,2> BoundingVolume::t() const{
    return t_;
}

}
