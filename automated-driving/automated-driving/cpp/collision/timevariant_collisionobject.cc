#include "collision/timevariant_collisionobject.h"

namespace collision{



bool TimeVariantCollisionObject::collide(const CollisionObject& c) const {
	return c.collide(*this);
}

bool TimeVariantCollisionObject::collide(const Shape& shape) const {
	return shape.collide(*this);
}

bool TimeVariantCollisionObject::collide(const Point& point) const {
    for (auto i : collision_object_at_time_) {
        if(i->collide(point)) {
            return true;
        };
    }
    return false;
}

bool TimeVariantCollisionObject::collide(const RectangleAABB& aabb) const {
    for (auto i : collision_object_at_time_) {
        if(i->collide(aabb)) {
            return true;
        };
    }
    return false;
}

bool TimeVariantCollisionObject::collide(const RectangleOBB& obb) const {
    for (auto i : collision_object_at_time_) {
        if(i->collide(obb)) {
            return true;
        };
    }
    return false;
}

bool TimeVariantCollisionObject::collide(const Sphere& sphere) const {
    for (auto i : collision_object_at_time_) {
        if(i->collide(sphere)) {
            return true;
        };
    }
    return false;
}

bool TimeVariantCollisionObject::collide(const Triangle& triangle) const {
    for (auto i : collision_object_at_time_) {
        if(i->collide(triangle)) {
            return true;
        };
    }
    return false;
}

bool TimeVariantCollisionObject::collide(const TimeVariantCollisionObject& tvco) const {
    int start_idx = std::max(time_start_idx_, tvco.time_start_idx_);
    int stop_idx = std::min(time_end_idx_, tvco.time_end_idx_);
    for (int i=start_idx; i<=stop_idx; i++) {
        if (getObstacleAtTime(i)->collide(*tvco.getObstacleAtTime(i))) {
            return true;
        }
    }
    return false;
}

bool TimeVariantCollisionObject::collide(const ShapeGroup& sg) const {
   for (auto i : collision_object_at_time_) {
        if(i->collide(sg)) {
            return true;
        };
    }
    return false; 
}

bool TimeVariantCollisionObject::collide(const Polygon& polygon) const {
   for (auto i : collision_object_at_time_) {
        if(i->collide(polygon)) {
            return true;
        };
    }
    return false; 
}

void TimeVariantCollisionObject::print(std::ostringstream &stream) const {
    stream << "Timevariant obstacle, time " << time_start_idx_ << "-" <<time_end_idx_ << std::endl;
    for (int i=0; i<collision_object_at_time_.size(); i++) {
        stream << "  " << i+time_start_idx_ << ":";
        collision_object_at_time_[i]->print(stream);
    }
}

BoundingVolumeConstPtr TimeVariantCollisionObject::getBoundingVolume() const {
    std::array<double,2> x = {INFINITY,-INFINITY};
    std::array<double,2> y = {INFINITY,-INFINITY};
    std::array<int,2> t = {time_start_idx_, time_end_idx_};

//    t_step = time_start_idx_; // nur fuer einzelne BVs
    for (auto &object : collision_object_at_time_) {
        BoundingVolumeConstPtr temp = object->getBoundingVolume();
        std::array<double,2> x_temp = temp->x();
        std::array<double,2> y_temp = temp->y();
        if (x_temp[0] < x[0]) { x[0] = x_temp[0]; }
        if (x_temp[1] > x[1]) { x[1] = x_temp[1]; }
        if (y_temp[0] < y[0]) { y[0] = y_temp[0]; }
        if (y_temp[1] > y[1]) { y[1] = y_temp[1]; }

//        // ein BV pro Zeitschritt?
//        std::array<double,2> t_temp = {t_step,t_step};
//        BoundingVolume* bvts = new BoundingVolume(x_temp,y_temp,t_temp,object);
//        std::shared_ptr<BoundingVolume> bvts_ptr = std::shared_ptr<BoundingVolume>(bvts);
//
//        // naechster Zeitschritt
//        t_step++;
    }

    BoundingVolume* bv = new BoundingVolume(x,y,t,shared_from_this());
    std::shared_ptr<BoundingVolume> bv_ptr = std::shared_ptr<BoundingVolume>(bv);
    return bv_ptr;
}

TimeVariantCollisionObject::TimeVariantCollisionObject(int time_start_idx) {
    time_start_idx_ = time_start_idx;
    time_end_idx_ = time_start_idx-1; 
}
    
CollisionObjectConstPtr TimeVariantCollisionObject::getObstacleAtTime(int time_idx) const {
    if (time_idx<time_start_idx_ || time_idx>time_end_idx_) return nullptr;
    return collision_object_at_time_[time_idx-time_start_idx_];
}

int TimeVariantCollisionObject::time_start_idx() {
    return time_start_idx_;
}

int TimeVariantCollisionObject::time_end_idx() {
    return time_end_idx_;
}

CollisionObjectConstPtr TimeVariantCollisionObject::timeSlice(int time_idx, CollisionObjectConstPtr shared_ptr_this) const  {
    return getObstacleAtTime(time_idx);
}
    
int TimeVariantCollisionObject::appendObstacle(CollisionObjectConstPtr obstacle) {
    collision_object_at_time_.push_back(obstacle);
    return ++time_end_idx_;
}


}