#include "collision_checker.h"

namespace collision {

void CollisionChecker::addCollisionObject(CollisionObjectConstPtr co) {
    collision_objects_.push_back(co);
}

 
bool CollisionChecker::collide(CollisionObjectConstPtr co) const {
    for( auto &c : collision_objects_ ) {
        if (c->collide(*co)) {
            return true;
        }
    }
  return false;
}
    
bool CollisionChecker::collide(CollisionObjectConstPtr co, CollisionObjectConstPtr &obstacle) const {
    for( auto &c : collision_objects_ ) {
        if (c->collide(*co)) {
            obstacle = c;
            return true;
        }
    }
    return false;
}
    
bool CollisionChecker::collide(CollisionObjectConstPtr co, std::vector<CollisionObjectConstPtr> &obstacles) const {
    bool collides = false;
    for( auto &c : collision_objects_ ) {
        if (c->collide(*co)) {
            obstacles.push_back(c);
            collides = true;
        }
    }
    return collides;
}
    
CollisionCheckerPtr CollisionChecker::windowQuery(const RectangleAABB& aabb) const {
    CollisionCheckerPtr cc_ret = std::shared_ptr<CollisionChecker>(new CollisionChecker());
    for( auto &c : collision_objects_ ) {
        if (c->collide(aabb)) {
            cc_ret->addCollisionObject(c);
        }
    }
    return cc_ret;
}
    
CollisionCheckerPtr CollisionChecker::timeSlice(int time_idx) const {
    CollisionCheckerPtr cc_ret(new CollisionChecker());
    for( auto &c : collision_objects_) {
        CollisionObjectConstPtr tmp = c->timeSlice(time_idx, c);
        if (tmp!=nullptr) {
            cc_ret->addCollisionObject(tmp);
        }
    }
    return cc_ret;
}

int CollisionChecker::numberOfObstacles() const {
    return collision_objects_.size();
}
    
std::vector<CollisionObjectConstPtr> CollisionChecker::getObstacles() const {
    return collision_objects_;
}
    
void CollisionChecker::print(std::ostringstream &stream) const {
    stream << "CollisionChecker number of CollisionObjects: " << collision_objects_.size() << std::endl;
    for (int i=0; i<collision_objects_.size();i++) {
        collision_objects_[i]->print(stream);
    }
}

}