#ifndef COLLISION_CHECKER_H_
#define COLLISION_CHECKER_H_

#include "collision_object.h"
#include "rectangle_aabb.h"
#include <vector>
#include <sstream>

namespace collision {

	class CollisionChecker;
	typedef std::shared_ptr<CollisionChecker> CollisionCheckerPtr;
	typedef std::shared_ptr<const CollisionChecker> CollisionCheckerConstPtr;

class CollisionChecker {
public:
	
  void addCollisionObject(CollisionObjectConstPtr co);
	bool collide(CollisionObjectConstPtr co) const;
	bool collide(CollisionObjectConstPtr co, CollisionObjectConstPtr &obstacle) const;
	bool collide(CollisionObjectConstPtr co, std::vector<CollisionObjectConstPtr> &obstacles) const;
	CollisionCheckerPtr windowQuery(const RectangleAABB& aabb) const;
	CollisionCheckerPtr timeSlice(int time_idx) const;
	void print(std::ostringstream &stream) const;
	int numberOfObstacles() const;
    std::vector<CollisionObjectConstPtr> getObstacles() const;


protected:
	std::vector<CollisionObjectConstPtr> collision_objects_;
};




}

#endif