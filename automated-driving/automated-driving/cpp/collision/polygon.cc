#include "collision/polygon.h"

namespace collision
{

Polygon::Polygon(std::vector<Eigen::Vector2d> &vertices,
          std::vector<std::vector<Eigen::Vector2d> > &hole_vertices,
          std::vector<TriangleConstPtr> &mesh_triangles) {
  vertices_ = vertices;
  hole_vertices_ = hole_vertices;
  mesh_triangles_ = mesh_triangles;
}

bool Polygon::collide(const CollisionObject &c) const {
  return c.collide(*this);
}

bool Polygon::collide(const Shape &shape) const {
  return shape.collide(*this);
}

bool Polygon::collide(const Point &point) const {
  for (auto i : mesh_triangles_) {
    if (i->collide(point)) {
      return true;
    };
  }
  return false;
}

bool Polygon::collide(const RectangleAABB &aabb) const {
  for (auto i : mesh_triangles_) {
    if (i->collide(aabb)) {
      return true;
    };
  }
  return false;
}

bool Polygon::collide(const RectangleOBB &obb) const {
  for (auto i : mesh_triangles_) {
    if (i->collide(obb)) {
      return true;
    };
  }
  return false;
}

bool Polygon::collide(const Sphere &sphere) const {
  for (auto i : mesh_triangles_) {
    if (i->collide(sphere)) {
      return true;
    };
  }
  return false;
}

bool Polygon::collide(const Triangle &triangle) const {
  for (auto i : mesh_triangles_) {
    if (i->collide(triangle)) {
      return true;
    };
  }
  return false;
}

bool Polygon::collide(const TimeVariantCollisionObject &tvco) const {
  for (auto i : mesh_triangles_) {
    if (i->collide(tvco)) {
      return true;
    };
  }
  return false;
}

bool Polygon::collide(const ShapeGroup &sg) const {
  for (auto i : mesh_triangles_) {
    if (i->collide(sg)) {
      return true;
    };
  }
  return false;
}

bool Polygon::collide(const Polygon &polygon) const {
  for (auto i : mesh_triangles_) {
    if (i->collide(polygon)) {
      return true;
    };
  }
  return false;
}

void Polygon::print(std::ostringstream &stream) const
{
}

BoundingVolumeConstPtr Polygon::getBoundingVolume() const {
  std::array<double, 2> x = {INFINITY, -INFINITY};
  std::array<double, 2> y = {INFINITY, -INFINITY};
  std::array<int, 2> t = {-1, -1};

  // get min/max from all included shapes
  for (auto &triangle : mesh_triangles_) {
    BoundingVolumeConstPtr temp = triangle->getBoundingVolume();
    std::array<double, 2> x_temp = temp->x();
    std::array<double, 2> y_temp = temp->y();
    if (x_temp[0] < x[0]) {
      x[0] = x_temp[0];
    }
    if (x_temp[1] > x[1]) {
      x[1] = x_temp[1];
    }
    if (y_temp[0] < y[0]) {
      y[0] = y_temp[0];
    }
    if (y_temp[1] > y[1]) {
      y[1] = y_temp[1];
    }
  }
  BoundingVolume *bv = new BoundingVolume(x, y, t, shared_from_this());
  std::shared_ptr<BoundingVolume> bv_ptr = std::shared_ptr<BoundingVolume>(bv);
  return bv_ptr;
}

CollisionObjectConstPtr Polygon::timeSlice(
  int time_idx, CollisionObjectConstPtr shared_ptr_this) const {
  return shared_ptr_this;
}

std::vector<TriangleConstPtr> Polygon::getTriangleMesh() const {
  return mesh_triangles_;
}

std::vector<Eigen::Vector2d> Polygon::getVertices() const {
  return vertices_;
}


}
