#ifndef COLLISION_OBJECT_H_
#define COLLISION_OBJECT_H_

#include <iostream>
#include <memory>
#include <sstream>


namespace collision{

class Shape;
class Point;
class RectangleAABB;
class RectangleOBB;
class Sphere;
class Triangle;
class TimeVariantCollisionObject;
class CollisionObject;
class ShapeGroup;
class Polygon;
class BoundingVolume;

typedef std::shared_ptr<CollisionObject> CollisionObjectPtr;
typedef std::shared_ptr<const CollisionObject> CollisionObjectConstPtr;

typedef std::shared_ptr<BoundingVolume> BoundingVolumePtr;
typedef std::shared_ptr<const BoundingVolume> BoundingVolumeConstPtr;

class CollisionObject : public std::enable_shared_from_this<CollisionObject> {
public:

    virtual ~CollisionObject() {}

    //! Dispatcher functions
    virtual bool collide(const CollisionObject& c) const = 0;
    virtual bool collide(const Shape& shape) const = 0;
    virtual bool collide(const Point& point) const = 0;
    virtual bool collide(const RectangleAABB& aabb) const = 0;
    virtual bool collide(const RectangleOBB& obb) const = 0;
    virtual bool collide(const Sphere& sphere) const = 0;
    virtual bool collide(const Triangle& triangle) const = 0;
    virtual bool collide(const TimeVariantCollisionObject& tvco) const = 0;
    virtual bool collide(const ShapeGroup& sg) const = 0;
    virtual bool collide(const Polygon& polygon) const = 0;

        //! Print all parameters of the shape
    virtual void print(std::ostringstream &stream) const = 0;
    virtual CollisionObjectConstPtr timeSlice(int time_idx, CollisionObjectConstPtr shared_ptr_this) const = 0;
    virtual BoundingVolumeConstPtr getBoundingVolume() const = 0;

};


typedef std::shared_ptr<Shape> ShapePtr;
typedef std::shared_ptr<const Shape> ShapeConstPtr;

typedef std::shared_ptr<RectangleAABB> RectangleAABBPtr;
typedef std::shared_ptr<const RectangleAABB> RectangleAABBConstPtr;

typedef std::shared_ptr<const RectangleOBB> RectangleOBBPtr;
typedef std::shared_ptr<const RectangleOBB> RectangleOBBConstPtr;

}

#endif
