#ifndef SHAPE_GROUP_H_
#define SHAPE_GROUP_H_

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "collision/collision_object.h"
#include "collision/shape.h"
#include "boundingvolume.h"

namespace collision{

class ShapeGroup : public CollisionObject {
public:

    virtual ~ShapeGroup() {}

    //! Dispatcher functions
    virtual bool collide(const CollisionObject& c) const;
    virtual bool collide(const Shape& shape) const;
    virtual bool collide(const Point& point) const;
    virtual bool collide(const RectangleAABB& aabb) const;
    virtual bool collide(const RectangleOBB& obb) const;
    virtual bool collide(const Sphere& sphere) const;
    virtual bool collide(const Triangle& triangle) const;
    virtual bool collide(const TimeVariantCollisionObject& tvco) const;
    virtual bool collide(const ShapeGroup& sg) const;
    bool collide(const Polygon& polygon) const;

    //! Print all parameters of the shape
    virtual void print(std::ostringstream &stream) const;
    virtual CollisionObjectConstPtr timeSlice(int time_idx, CollisionObjectConstPtr shared_ptr_this) const ;
    BoundingVolumeConstPtr getBoundingVolume() const;

    void addToGroup(ShapeConstPtr shape);
    std::vector<ShapeConstPtr> unpack() const;

protected:
    std::vector<ShapeConstPtr> shapes_;
};

typedef std::shared_ptr<ShapeGroup> ShapeGroupPtr;
typedef std::shared_ptr<const ShapeGroup> ShapeGroupConstPtr;

}

#endif
