#include "triangle.h"
#include "collision/polygon.h"
#include "primitive_collision_tests.h"

namespace collision {

bool Triangle::collide(const CollisionObject& c) const {
  return c.collide(*this);
}

bool Triangle::collide(const Shape& shape) const {
    return shape.collide(*this);
}

bool Triangle::collide(const Point& point) const {
    return primitive_tests::collisionDetection(point, *this);
}

bool Triangle::collide(const RectangleAABB& aabb) const {
    return primitive_tests::collisionDetection(aabb, *this);
}

bool Triangle::collide(const RectangleOBB& obb) const {
    return primitive_tests::collisionDetection(obb, *this);
}

bool Triangle::collide(const Sphere& sphere) const {
    return primitive_tests::collisionDetection(sphere, *this);
}

bool Triangle::collide(const Triangle& triangle) const {
    return primitive_tests::collisionDetection(*this, triangle);
}

bool Triangle::collide(const TimeVariantCollisionObject& tvco) const {
    return tvco.collide(*this);
}

bool Triangle::collide(const ShapeGroup& sg) const {
  return sg.collide(*this);
}

bool Triangle::collide(const Polygon& polygon) const {
  return polygon.collide(*this);
}

Triangle* Triangle::clone() const {
     return new Triangle(*this);
}

Triangle::Triangle(const Triangle& copy)
    : Shape(copy),
      v1_(copy.v1()),
      v2_(copy.v2()),
      v3_(copy.v3())
{
    center_ = copy.center();
    radius_ = copy.radius();
}

ShapeType Triangle::type() {
    return type_;
}

void Triangle::print(std::ostringstream &stream) const {
    stream << "Triangle: \nVertices:"
              << "(" << v1_(0) << "|" << v1_(1) << "), "
              << "(" << v2_(0) << "|" << v2_(1) << "), "
              << "(" << v3_(0) << "|" << v3_(1) << "), "
              << "\ncenter: "
              << "(" << center_x() << "|" << center_y() << "), "
              << std::endl;
}

BoundingVolumeConstPtr Triangle::getBoundingVolume() const {
    std::array<double,2> x = {v1_[0],v1_[0]};
    std::array<double,2> y = {v1_[1],v1_[1]};
    std::array<int,2> t = {-1, -1};

//  quick and (very) dirty, TBD wenn es genutzt werden soll
    if (v2_[0] < x[0]) { x[0] = v2_[0]; }
    if (v2_[0] > x[1]) { x[1] = v2_[0]; }
    if (v2_[1] < y[0]) { y[0] = v2_[1]; }
    if (v2_[1] > y[1]) { y[1] = v2_[1]; }

    if (v3_[0] < x[0]) { x[0] = v3_[0]; }
    if (v3_[0] > x[1]) { x[1] = v3_[0]; }
    if (v3_[1] < y[0]) { y[0] = v3_[1]; }
    if (v3_[1] > y[1]) { y[1] = v3_[1]; }

    BoundingVolume* bv = new BoundingVolume(x,y,t,shared_from_this());
    std::shared_ptr<BoundingVolume> bv_ptr = std::shared_ptr<BoundingVolume>(bv);
    return bv_ptr;
}

Eigen::Vector2d Triangle::v1() const {
    return v1_;
}

Eigen::Vector2d Triangle::v2() const {
    return v2_;
}

Eigen::Vector2d Triangle::v3() const {
    return v3_;
}


void Triangle::set_v1(Eigen::Vector2d _v1) {
    v1_ = _v1;
}

void Triangle::set_v2(Eigen::Vector2d _v2) {
    v2_ = _v2;
}

void Triangle::set_v3(Eigen::Vector2d _v3) {
    v3_ = _v3;
}

Eigen::Vector2d Triangle::compute_center() {
    double x = (v1_(0) + v2_(0) + v3_(0))/3.0;
    double y = (v1_(1) + v2_(1) + v3_(1))/3.0;
    return Eigen::Vector2d(x,y);
}

}
