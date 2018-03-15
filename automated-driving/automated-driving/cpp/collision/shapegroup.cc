#include "collision/shapegroup.h"

namespace collision {

bool ShapeGroup::collide(const CollisionObject& c) const {
  return c.collide(*this);
}

bool ShapeGroup::collide(const Shape& shape) const {
  return shape.collide(*this);
}

bool ShapeGroup::collide(const Point& point) const {
  for (auto i : shapes_) {
    if(i->collide(point)) {
      return true;
    };
  }
  return false;
}

bool ShapeGroup::collide(const RectangleAABB& aabb) const {
  for (auto i : shapes_) {
    if(i->collide(aabb)) {
      return true;
    };
  }
  return false;
}

bool ShapeGroup::collide(const RectangleOBB& obb) const {
  for (auto i : shapes_) {
    if(i->collide(obb)) {
      return true;
    };
  }
  return false;
}

bool ShapeGroup::collide(const Sphere& sphere) const {
  for (auto i : shapes_) {
    if(i->collide(sphere)) {
      return true;
    };
  }
  return false;
}

bool ShapeGroup::collide(const Triangle& triangle) const {
  for (auto i : shapes_) {
    if(i->collide(triangle)) {
      return true;
      };
  }
  return false;
}

bool ShapeGroup::collide(const TimeVariantCollisionObject& tvco) const {
  for (auto i : shapes_) {
    if(i->collide(tvco)) {
      return true;
    };
  }
  return false;
}

bool ShapeGroup::collide(const ShapeGroup& sg) const {
  for (auto &shape : shapes_) {
    for (auto &shape2 : sg.shapes_) {
      if (shape->collide(*shape2)) {
        return true;
      }
    }
  }
  return false;
}

bool ShapeGroup::collide(const Polygon& polygon) const {
  for (auto i : shapes_) {
    if(i->collide(polygon)) {
      return true;
    };
  }
  return false;
}

void ShapeGroup::print(std::ostringstream &stream) const {

}

BoundingVolumeConstPtr ShapeGroup::getBoundingVolume() const {
  std::array<double,2> x = {INFINITY,-INFINITY};
  std::array<double,2> y = {INFINITY,-INFINITY};
  std::array<int,2> t = {-1, -1};

    // get min/max from all included shapes
  for (auto &shape : shapes_) {
    BoundingVolumeConstPtr temp = shape->getBoundingVolume();
    std::array<double,2> x_temp = temp->x();
    std::array<double,2> y_temp = temp->y();
    if (x_temp[0] < x[0]) { x[0] = x_temp[0]; }
    if (x_temp[1] > x[1]) { x[1] = x_temp[1]; }
    if (y_temp[0] < y[0]) { y[0] = y_temp[0]; }
    if (y_temp[1] > y[1]) { y[1] = y_temp[1]; }
  }

  BoundingVolume* bv = new BoundingVolume(x,y,t,shared_from_this());
  std::shared_ptr<BoundingVolume> bv_ptr = std::shared_ptr<BoundingVolume>(bv);
  return bv_ptr;
}

CollisionObjectConstPtr ShapeGroup::timeSlice(
  int time_idx, CollisionObjectConstPtr shared_ptr_this) const {
	return shared_ptr_this;
}

void ShapeGroup::addToGroup(ShapeConstPtr shape) {
	shapes_.push_back(shape);
}

std::vector<ShapeConstPtr> ShapeGroup::unpack() const {
  return shapes_;
}

    
}
