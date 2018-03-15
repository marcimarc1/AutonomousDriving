#ifndef RECTANGLE_OBB_H_
#define RECTANGLE_OBB_H_

#include <iostream>
#include <exception>

#include "shape.h"

namespace collision {

class RectangleOBB : public Shape {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RectangleOBB(double _r_x, double _r_y, Eigen::Matrix2d _local_axes, const Eigen::Vector2d &_center = Eigen::Vector2d(0,0))
                    : Shape(_center), local_axes_(_local_axes), r_(_r_x, _r_y)
    {
    }

    RectangleOBB(double _r_x, double _r_y, double angle, const Eigen::Vector2d &_center = Eigen::Vector2d(0,0))
                    : Shape(_center), r_(_r_x, _r_y)
    {
        local_axes_ << cos(angle), -sin(angle), sin(angle), cos(angle);
    }

    virtual ~RectangleOBB() {}

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

    RectangleOBB( const RectangleOBB& copy);
    virtual RectangleOBB* clone() const;

    void print(std::ostringstream &stream) const;
    BoundingVolumeConstPtr getBoundingVolume() const;

    ShapeType type();

    Eigen::Matrix2d local_axes() const;
    Eigen::Vector2d local_x_axis() const;
    Eigen::Vector2d local_y_axis() const;
    Eigen::Vector2d r() const;
    double r(int i) const;
    double r_x() const;
    double r_y() const;

    void set_local_x_axis(Eigen::Vector2d x_axis);
    void set_local_y_axis(Eigen::Vector2d y_axis);
    void set_r_x(double _r_x);
    void set_r_y(double _r_y);

    /**
     * @brief Computes the square distance between a point and
     *        the rectangle boundary. From:
     *        C. Ericson, Real-Time Collision Detection, pp. 134, 2004
     * @param p Point
     */
    double squareDisToPoint(const Eigen::Vector2d &p) const;

protected:
    using Shape::center_;
    using Shape::radius_;

    /** Local x- and y-axis (column vectors)
     *               | x_1  x_2 |
     *  local_axis = |          |
     *               | y_1  y_2 |
     */
    Eigen::Matrix2d local_axes_;

    //! Positive halfwidth extents of OBB along each axis
    Eigen::Vector2d r_;

    static constexpr ShapeType type_ = TYPE_RECTANGLE_OBB;
};

}

#endif
