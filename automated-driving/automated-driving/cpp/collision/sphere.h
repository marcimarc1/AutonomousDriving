#ifndef SPHERE_H_
#define SPHERE_H_

#include <Eigen/Dense>
#include <iostream>

#include "shape.h"

namespace collision{

class Sphere : public Shape{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Sphere(double _radius, const Eigen::Vector2d &_center = Eigen::Vector2d(0,0))
                    : Shape(_center)
    {
        radius_ = _radius;
    }

    Sphere(double _radius, double x, double y)
                    : Shape(Eigen::Vector2d(x,y))
    {
        radius_ = _radius;
    }

    ~Sphere() {}

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
    

    Sphere(const Sphere& copy);
    Sphere* clone() const;

    void print(std::ostringstream &stream) const;
    BoundingVolumeConstPtr getBoundingVolume() const;

    void set_radius(double _radius);
    double radius() const {return radius_;};
    double get_x() const {return center_(0);};
    double get_y() const {return center_(1);};


    ShapeType type();

protected:


protected:
    using Shape::center_;
    using Shape::radius_;

    static constexpr ShapeType type_ = TYPE_SPHERE;
};

}

#endif
