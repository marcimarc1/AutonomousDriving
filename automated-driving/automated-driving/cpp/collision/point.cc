#include "point.h"
#include "collision/polygon.h"
#include "primitive_collision_tests.h"

namespace collision {

bool Point::collide(const CollisionObject& c) const {
  return c.collide(*this);
}

bool Point::collide(const Shape& shape) const {
    return shape.collide(*this);
}

bool Point::collide(const Point& point) const {
    return primitive_tests::collisionDetection(point, *this);
}

bool Point::collide(const RectangleAABB& aabb) const {
    return primitive_tests::collisionDetection(*this, aabb);
}

bool Point::collide(const RectangleOBB& obb) const {
    return primitive_tests::collisionDetection(*this, obb);
}

bool Point::collide(const Sphere& sphere) const {
    return primitive_tests::collisionDetection(*this, sphere);
}

bool Point::collide(const Triangle& triangle) const {
    return primitive_tests::collisionDetection(*this, triangle);
}

bool Point::collide(const TimeVariantCollisionObject& tvco) const {
    return tvco.collide(*this);
}

bool Point::collide(const ShapeGroup& sg) const {
  return sg.collide(*this);
}

bool Point::collide(const Polygon& polygon) const {
  return polygon.collide(*this);
}

Point* Point::clone() const {
     return new Point(*this);
}

Point::Point(const Point& copy)
    : Shape(copy) {
}

void Point::print(std::ostringstream &stream) const{
    stream << "Point: center: ("
              << center_x() << "/" << center_y()
              << ")"
              << std::endl;
}

BoundingVolumeConstPtr Point::getBoundingVolume() const {
    std::array<double,2> x = {center_x(), center_x()};
    std::array<double,2> y = {center_y(), center_y()};
    std::array<int,2> t = {-1, -1};
    BoundingVolume* bv = new BoundingVolume(x,y,t,shared_from_this());
    std::shared_ptr<BoundingVolume> bv_ptr = std::shared_ptr<BoundingVolume>(bv);
    return bv_ptr;
}

ShapeType Point::type() {
    return type_;
}
}
