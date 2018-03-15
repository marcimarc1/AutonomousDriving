#ifndef SHAPE_H_
#define SHAPE_H_

#include <iostream>
#include <Eigen/Dense>

#include "collision/collision_object.h"
#include "collision/timevariant_collisionobject.h"
#include "collision/shapegroup.h"
//#include "collision/polygon.hpp"

namespace collision{

enum ShapeType {
    TYPE_POINT,
    TYPE_RECTANGLE_AABB,
    TYPE_RECTANGLE_OBB,
    TYPE_SPHERE,
    TYPE_TRIANGLE
};

typedef ShapeType ShapeType;

//! Base prototype for the shape of an obstacle
class Shape : public CollisionObject {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    Shape(const Shape& copy);
    virtual Shape* clone() const = 0;

    //! Get geometric center of shape
    Eigen::Vector2d center() const;
    double center_x() const;
    double center_y() const;

    //! Set geometric center of shape
    void set_center(const Eigen::Vector2d& _center);

    //! Print all parameters of the shape
    virtual void print(std::ostringstream &stream) const = 0;
    virtual CollisionObjectConstPtr timeSlice(int time_idx, CollisionObjectConstPtr shared_ptr_this) const;

    //! Get shape type
    virtual ShapeType type() = 0;

    //! Get radius
    double radius() const;

    virtual ~Shape();
    
protected:
    Shape(const Eigen::Vector2d& _center)
        : center_(_center)
    {
    }



protected:
    Eigen::Vector2d center_;
    double radius_;
};



}

#endif
