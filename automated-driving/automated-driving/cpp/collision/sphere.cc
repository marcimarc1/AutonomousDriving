#include "collision/sphere.h"
#include "collision/polygon.h"
#include "primitive_collision_tests.h"

namespace collision {

//! Dispatcher functions
bool Sphere::collide(const CollisionObject& c) const {
  return c.collide(*this);
}

bool Sphere::collide(const Shape& shape) const {
    return shape.collide(*this);
}

bool Sphere::collide(const Point& point) const {
    return primitive_tests::collisionDetection(point, *this);
}

bool Sphere::collide(const RectangleAABB& aabb) const {
    return primitive_tests::collisionDetection(aabb, *this);
}

bool Sphere::collide(const RectangleOBB& obb) const {
    return primitive_tests::collisionDetection(obb, *this);
}

bool Sphere::collide(const Sphere& sphere) const {
    return primitive_tests::collisionDetection(*this, sphere);
}

bool Sphere::collide(const Triangle& triangle) const {
    return primitive_tests::collisionDetection(*this, triangle);
}

bool Sphere::collide(const TimeVariantCollisionObject& tvco) const {
    return tvco.collide(*this);
}

bool Sphere::collide(const ShapeGroup& sg) const {
  return sg.collide(*this);
}

bool Sphere::collide(const Polygon& polygon) const {
  return polygon.collide(*this);
}

Sphere* Sphere::clone() const {
     return new Sphere(*this);
}

Sphere::Sphere(const Sphere& copy) : Shape(copy) {
    center_ = copy.center();
    radius_ = copy.radius();
}

void Sphere::print(std::ostringstream &stream) const{
    stream << "Sphere:\n"
              << "center: (" << center_x() << "|" << center_y() << ")\n"
              << "radius: " << radius_
              << std::endl;
}

BoundingVolumeConstPtr Sphere::getBoundingVolume() const {
    std::array<double,2> x = {center_[0] - radius_, center_[0] + radius_};
    std::array<double,2> y = {center_[1] - radius_, center_[1] + radius_};
    std::array<int,2> t = {-1, -1};
    BoundingVolume* bv = new BoundingVolume(x,y,t,shared_from_this());
    std::shared_ptr<BoundingVolume> bv_ptr = std::shared_ptr<BoundingVolume>(bv);
    return bv_ptr;
}

void Sphere::set_radius(double _radius) {
    radius_ = _radius;
}

ShapeType Sphere::type() {
    return type_;
}

}
