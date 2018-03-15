#ifndef TIME_VARIANT_COLLISION_OBJECT_H_
#define TIME_VARIANT_COLLISION_OBJECT_H_

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

//#include "collision/rectangle_obb.hpp"
//#include "collision/rectangle_aabb.hpp"
#include "collision/collision_object.h"
#include "collision/shape.h"
#include "collision/shapegroup.h"

namespace collision{

class TimeVariantCollisionObject : public CollisionObject {
public:

    virtual ~TimeVariantCollisionObject() {}

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

    TimeVariantCollisionObject(int time_start_idx);
    CollisionObjectConstPtr getObstacleAtTime(int time_idx) const;
    int appendObstacle(CollisionObjectConstPtr obstacle);

    int time_start_idx();
    int time_end_idx();

protected:
    int time_start_idx_;
    int time_end_idx_;
    std::vector<CollisionObjectConstPtr> collision_object_at_time_;
};

typedef std::shared_ptr<TimeVariantCollisionObject> TimeVariantCollisionObjectPtr;
typedef std::shared_ptr<const TimeVariantCollisionObject> TimeVariantCollisionObjectConstPtr;

}

#endif
