#ifndef POLYGON_H_
#define POLYGON_H_

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "collision/collision_object.h"
#include "collision/triangle.h"
#include "boundingvolume.h"

namespace collision{

class Polygon : public CollisionObject {
public:

  Polygon(std::vector<Eigen::Vector2d> &vertices,
          std::vector<std::vector<Eigen::Vector2d> > &hole_vertices,
          std::vector<TriangleConstPtr> &mesh_triangles);
  virtual ~Polygon() {}

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
  virtual bool collide(const Polygon& polygon) const;

  //! Print all parameters of the shape
  virtual void print(std::ostringstream &stream) const;
  virtual CollisionObjectConstPtr timeSlice(
    int time_idx, CollisionObjectConstPtr shared_ptr_this) const;
  BoundingVolumeConstPtr getBoundingVolume() const;

  std::vector<TriangleConstPtr> getTriangleMesh() const;
  std::vector<Eigen::Vector2d> getVertices() const;

protected:
  std::vector<Eigen::Vector2d> vertices_;
  std::vector<std::vector<Eigen::Vector2d> > hole_vertices_;
  std::vector<TriangleConstPtr> mesh_triangles_;
};

typedef std::shared_ptr<Polygon> PolygonPtr;
typedef std::shared_ptr<const Polygon> PolygonConstPtr;

}

#endif
