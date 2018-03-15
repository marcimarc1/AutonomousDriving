#ifndef POINT_H_
#define POINT_H_

#include <Eigen/Dense>
#include <iostream>

#include "shape.h"
#include "boundingvolume.h"

namespace collision{

class Point : public Shape{

public:
    Point(const Eigen::Vector2d &_center = Eigen::Vector2d(0,0))
                    : Shape(_center)
    {
    }

    ~Point() {}

    //! Dispatcher functions
    bool collide(const CollisionObject& c) const ;
    bool collide(const Shape& shape) const;
    bool collide(const Point& point) const;
    bool collide(const RectangleAABB& aabb) const;
    bool collide(const RectangleOBB& obb) const;
    bool collide(const Sphere& sphere) const;
    bool collide(const Triangle& triangle) const;
    bool collide(const TimeVariantCollisionObject& tvco) const;
    bool collide(const ShapeGroup& sg) const;
    bool collide(const Polygon& polygon) const;

    Point(const Point& copy);
    Point* clone() const;

    void print(std::ostringstream &stream) const;
    BoundingVolumeConstPtr getBoundingVolume() const;

    ShapeType type();

protected:
    using Shape::center_;
    using Shape::radius_;

    static constexpr ShapeType type_ = TYPE_POINT;
};

}

#endif
