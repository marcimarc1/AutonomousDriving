#include "rectangle_aabb.h"
#include "collision/polygon.h"
#include "primitive_collision_tests.h"

namespace collision {

//! Dispatcher functions
bool RectangleAABB::collide(const CollisionObject& c) const {
  return c.collide(*this);
}

bool RectangleAABB::collide(const Shape& shape) const {
    return shape.collide(*this);
}

bool RectangleAABB::collide(const Point& point) const {
    return primitive_tests::collisionDetection(point, *this);
}

bool RectangleAABB::collide(const RectangleAABB& aabb) const {
    return primitive_tests::collisionDetection(aabb, *this);
}

bool RectangleAABB::collide(const RectangleOBB& obb) const {
    return primitive_tests::collisionDetection(*this, obb);
}

bool RectangleAABB::collide(const Sphere& sphere) const {
    return primitive_tests::collisionDetection(*this, sphere);
}

bool RectangleAABB::collide(const Triangle& triangle) const {
    return primitive_tests::collisionDetection(*this, triangle);
}

bool RectangleAABB::collide(const TimeVariantCollisionObject& tvco) const {
    return tvco.collide(*this);
}

bool RectangleAABB::collide(const ShapeGroup& sg) const {
  return sg.collide(*this);
}

bool RectangleAABB::collide(const Polygon& polygon) const {
  return polygon.collide(*this);
}


RectangleAABB* RectangleAABB::clone() const {
     return new RectangleAABB(*this);
}

RectangleAABB::RectangleAABB(const RectangleAABB& copy)
    : Shape(copy) {
    center_ = copy.center(); // not needed? It is already set in Shape(copy)
    radius_ = copy.radius();
    r_ = copy.r();
    min_ = copy.min();
    max_ = copy.max();
}

ShapeType RectangleAABB::type() {
    return type_;
}

void RectangleAABB::print(std::ostringstream &stream) const {
    stream << "AABB Rectangle: center: ("
              << center_x() << "/" << center_y()
              << ") r: (" << r_(0) << "|" << r_(1) << ") "
              << "min: (" << min_(0) << "|" << min_(1) << ") "
              << "max: (" << max_(0) << "|" << max_(1) << ")"
              << std::endl;
}

BoundingVolumeConstPtr RectangleAABB::getBoundingVolume() const {
    std::array<double,2> x = {min_(0), max_(0)};
    std::array<double,2> y = {min_(1), max_(1)};
    std::array<int,2> t = {-1, -1};
    BoundingVolume* bv = new BoundingVolume(x,y,t,shared_from_this());
    std::shared_ptr<BoundingVolume> bv_ptr = std::shared_ptr<BoundingVolume>(bv);
    return bv_ptr;
}

Eigen::Vector2d RectangleAABB::min() const {
    return min_;
}

Eigen::Vector2d RectangleAABB::max() const {
    return max_;
}


Eigen::Vector2d RectangleAABB::r() const {
    return r_;
}

double RectangleAABB::r(int i) const {
    switch(i) {
    case 0:
        return r_(0);
    case 1:
        return r_(1);
    default:
        throw "Rectangle_OBB: Not a valid index for r";
    }
}

void RectangleAABB::set_r(const Eigen::Vector2d& _r) {
    r_ = _r;
    min_ = center_ - r_;
    max_ = center_ + r_;
}

double RectangleAABB::r_x() const {
    return r_(0);
}

double RectangleAABB::r_y() const {
    return r_(1);
}

void RectangleAABB::set_center(const Eigen::Vector2d& _center) {
    center_ = _center;
    min_ = center_ - r_;
    max_ = center_ + r_;
}

void RectangleAABB::set_r_x(double _r_x) {
    r_(0) = _r_x;
    min_(0) = center_(0) - r_(0);
    max_(0) = center_(0) + r_(0);
}

void RectangleAABB::set_r_y(double _r_y) {
    r_(1) = _r_y;
    min_(1) = center_(1) - r_(1);
    max_(1) = center_(1) + r_(1);
}

void RectangleAABB::set_all(double r_x, double r_y, double center_x, double center_y) {
    center_(0) = center_x;
    r_(0) = r_x;
    min_(0) = center_x - r_x;
    max_(0) = center_x + r_x;   

    center_(1) = center_y;
    r_(1) = r_y;
    min_(1) = center_y - r_y;
    max_(1) = center_y + r_y;
}

double RectangleAABB::squareDisToPoint(const Eigen::Vector2d& p) const {
    double sq_dis = 0.0;
    for(int i=0; i<2; i++) {
        if(p(i) < min_(i)) {
            sq_dis += pow(min_(i) - p(i),2);
        }
        else if(p(i) > max_(i)) {
            sq_dis += pow(p(i) - max_(i),2);
        }
    }
    return sq_dis;
}

}
