#ifndef RECTANGLE_AABB_H_
#define RECTANGLE_AABB_H_

#include <iostream>
#include <Eigen/Dense>

#include "shape.h"

namespace collision{

class RectangleAABB : public Shape {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RectangleAABB(double _rx, double _ry, const Eigen::Vector2d &_center = Eigen::Vector2d(0,0))
                    : Shape(_center), r_(_rx, _ry)
    {
        min_ = center_ - r_;
        max_ = center_ + r_;
    }

    ~RectangleAABB() {}

    //! Dispatcher functions
    bool collide(const CollisionObject& c) const;
    bool collide(const Shape& shape) const;
    bool collide(const Point& point) const;
    bool collide(const RectangleAABB& aabb) const;
    bool collide(const RectangleOBB& obb) const;
    bool collide(const Sphere& sphere) const;
    bool collide(const Triangle& triangle) const;
    bool collide(const TimeVariantCollisionObject& tvco) const;
    bool collide(const ShapeGroup& sg) const;
    bool collide(const Polygon& polygon) const;

    RectangleAABB( const RectangleAABB& copy);
    RectangleAABB* clone() const;

    void print(std::ostringstream &stream) const;
    BoundingVolumeConstPtr getBoundingVolume() const;

    ShapeType type();

    Eigen::Vector2d r() const;
    double r(int i) const;
    double r_x() const;
    double r_y() const;
    Eigen::Vector2d min() const;
    Eigen::Vector2d max() const;

    void set_center(const Eigen::Vector2d& _center);
    void set_r(const Eigen::Vector2d& _r);
    void set_r_x(double _r_x);
    void set_r_y(double _r_y);
    void set_all(double r_x, double r_y, double center_x, double center_y);

    /**
     * @brief Computes the square distance between a point and
     *        the rectangle boundary. From:
     *        C. Ericson, Real-Time Collision Detection, pp. 131, 2004
     * @param p Point
     */
    double squareDisToPoint(const Eigen::Vector2d &p) const;

protected:
    /** Center-radius representation
     /  ----------- max
     /  |    |ry  |
     /  |   c|____|
     /  |     ry  |
     /  |         |
     /  -----------
     / min
     */
    using Shape::center_;
    using Shape::radius_;

    //! Positive halfwidth extents of OBB along each axis (rx, ry)
    Eigen::Vector2d r_;

    //! Min-Max Points on rectangle
    Eigen::Vector2d min_;
    Eigen::Vector2d max_;

    static constexpr ShapeType type_ = TYPE_RECTANGLE_AABB;
};

}

#endif
