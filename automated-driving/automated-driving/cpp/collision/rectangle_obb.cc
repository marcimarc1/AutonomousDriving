#include "rectangle_obb.h"
#include "collision/polygon.h"
#include "primitive_collision_tests.h"

namespace collision {

//! Dispatcher functions
bool RectangleOBB::collide(const CollisionObject& c) const {
  return c.collide(*this);
}

bool RectangleOBB::collide(const Shape& shape) const {
    return shape.collide(*this);
}

bool RectangleOBB::collide(const Point& point) const {
    return primitive_tests::collisionDetection(point, *this);
}

bool RectangleOBB::collide(const RectangleAABB& aabb) const {
    return primitive_tests::collisionDetection(aabb, *this);
}

bool RectangleOBB::collide(const RectangleOBB& obb) const {
    return primitive_tests::collisionDetection(*this, obb);
}

bool RectangleOBB::collide(const Sphere& sphere) const {
    return primitive_tests::collisionDetection(*this, sphere);
}

bool RectangleOBB::collide(const Triangle& triangle) const {
    return primitive_tests::collisionDetection(*this, triangle);
}

bool RectangleOBB::collide(const TimeVariantCollisionObject& tvco) const {
    return tvco.collide(*this);
}

bool RectangleOBB::collide(const ShapeGroup& sg) const {
  return sg.collide(*this);
}

bool RectangleOBB::collide(const Polygon& polygon) const {
  return polygon.collide(*this);
}

RectangleOBB* RectangleOBB::clone() const {
     return new RectangleOBB(*this);
}

RectangleOBB::RectangleOBB(const RectangleOBB& copy)
    : Shape(copy) {
    center_ = copy.center();
    radius_ = copy.radius();
    local_axes_ = copy.local_axes();
    r_ = copy.r();
}

ShapeType RectangleOBB::type() {
    return type_;
}

void RectangleOBB::print(std::ostringstream &stream) const {
    stream << "OBB Rectangle: center: ("
              << center_x() << "/" << center_y()
              << "), r: (" << r_(0) << "|" << r_(1) << ") "
              << "Local coordinate axes: (" << local_axes_(0,0) << "," << local_axes_(1,0)
              << "), (" <<  local_axes_(0,1) << "," << local_axes_(1,1) << ")"
              << std::endl;
}

BoundingVolumeConstPtr RectangleOBB::getBoundingVolume() const {

    std::array<double,2> x = {center_[0],center_[0]};
    std::array<double,2> y = {center_[1],center_[1]};
    std::array<int,2> t = {-1, -1};

    //define positions of the 4 corners (relative to the center point and aligned to the local axes!)
    std::array<Eigen::Vector2d,4> r_points;
    r_points[0] = r_;
    r_points[1] = r_points[0];
    r_points[1][1] = -r_points[1][1];
    r_points[2] = r_points[1] * -1;
    r_points[3] = r_points[0] * -1;

    // calculate min/max for each of the 4 corners
    for (int i=0; i<4; i++) {
        Eigen::Vector2d corner_i = center() + local_axes() * r_points[i];
        if (corner_i[0] < x[0]) { x[0] = corner_i[0]; }
        if (corner_i[0] > x[1]) { x[1] = corner_i[0]; }

        if (corner_i[1] < y[0]) { y[0] = corner_i[1]; }
        if (corner_i[1] > y[1]) { y[1] = corner_i[1]; }
    }

    BoundingVolume* bv = new BoundingVolume(x,y,t,shared_from_this());
    std::shared_ptr<BoundingVolume> bv_ptr = std::shared_ptr<BoundingVolume>(bv);
    return bv_ptr;
}

Eigen::Matrix2d RectangleOBB::local_axes() const {
    return local_axes_;
}

Eigen::Vector2d RectangleOBB::local_x_axis() const {
    return local_axes_.col(0);
}

Eigen::Vector2d RectangleOBB::local_y_axis() const {
    return local_axes_.col(1);
}

Eigen::Vector2d RectangleOBB::r() const {
    return r_;
}

double RectangleOBB::r(int i) const {
    switch(i) {
    case 0:
        return r_(0);
    case 1:
        return r_(1);
    default:
        throw "Rectangle_OBB: Not a valid index for r";
    }
}

double RectangleOBB::r_x() const {
    return r_(0);
}

double RectangleOBB::r_y() const {
    return r_(1);
}

void RectangleOBB::set_local_x_axis(Eigen::Vector2d x_axis) {
    local_axes_.col(0) = x_axis;
}

void RectangleOBB::set_local_y_axis(Eigen::Vector2d y_axis) {
    local_axes_.col(1) = y_axis;
}

void RectangleOBB::set_r_x(double _r_x) {
    r_(0) = _r_x;
}

void RectangleOBB::set_r_y(double _r_y) {
    r_(1) = _r_y;
}

double RectangleOBB::squareDisToPoint(const Eigen::Vector2d &p) const {
    double sq_dis = 0.0;
    //! Project translation vector t in OBB's local coordinate system
    Eigen::Vector2d t = local_axes_.transpose()*(p - center_);

    for(int i=0; i<2; i++) {
        if(t(i) < -r_(i)) {
            sq_dis += pow(t(i)+r_(i),2);
        }
        else if(t(i) > r_(i)) {
            sq_dis += pow(t(i)-r_(i),2);
        }
    }
    return sq_dis;
}

}
