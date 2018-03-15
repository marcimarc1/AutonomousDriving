#ifndef TRIANGLE_H_
#define TRIANGLE_H_

#include <iostream>
#include <Eigen/Dense>
#include <assert.h>

#include "shape.h"

namespace collision {


class Triangle : public Shape {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Triangle(const Eigen::Vector2d &_v1 = Eigen::Vector2d(0,0), 
             const Eigen::Vector2d &_v2 = Eigen::Vector2d(0,0), 
             const Eigen::Vector2d &_v3 = Eigen::Vector2d(0,0))
        : Shape(Eigen::Vector2d(0,0)), v1_(_v1), v2_(_v2), v3_(_v3)
    {
        set_center(compute_center());
    }

    ~Triangle() {}

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

    //! Copy function for triangle
    Triangle(const Triangle& copy);
    Triangle* clone() const;

    //! Print all parameters of the triangle
    void print(std::ostringstream &stream) const;
    BoundingVolumeConstPtr getBoundingVolume() const;

    //! Get shape type
    ShapeType type();

    //! Get member variables
    Eigen::Vector2d v1() const;
    Eigen::Vector2d v2() const;
    Eigen::Vector2d v3() const;

    //! ToDo
    double radius() const {assert(false); return 0;}

    //! Set member variables
    void set_v1(Eigen::Vector2d _v1);
    void set_v2(Eigen::Vector2d _v2);
    void set_v3(Eigen::Vector2d _v3);

private:
    Eigen::Vector2d compute_center();

private:
    using Shape::center_;
    using Shape::radius_;

    Eigen::Vector2d v1_;
    Eigen::Vector2d v2_;
    Eigen::Vector2d v3_;

    static constexpr ShapeType type_ = TYPE_TRIANGLE;

};


typedef std::shared_ptr<Triangle> TrianglePtr;
typedef std::shared_ptr<const Triangle> TriangleConstPtr;

}

#endif
