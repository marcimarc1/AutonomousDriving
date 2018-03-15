#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <vector>
#include <Eigen/Dense>

#include "collision/rectangle_aabb.h"
#include "collision/rectangle_obb.h"
#include "collision/collision_checker.h"
#include "collision/shapegroup.h"
#include "collision/polygon.h"
#include "collision/sphere.h"
#include "collision/triangle.h"
#include "collision/point.h"
#include "collision/trajectory.h"
#include "collision/boundingvolume_util.h"

namespace py = pybind11;

PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void init_module_collision(py::module &m) {

	py::class_<collision::CollisionObject,
             std::shared_ptr<collision::CollisionObject> >(m,
                                                           "CollisionObject");


  py::class_<collision::Shape, std::shared_ptr<collision::Shape> >(
      m, "Shape", py::base<collision::CollisionObject>());


  py::class_<collision::Point, std::shared_ptr<collision::Point> >(
      m, "Point", py::base<collision::Shape>())
    .def(py::init<>())
    .def("__init__", [](collision::Point &instance, double x, double y) {
      new (&instance) collision::Point(Eigen::Vector2d(x,y));
    })
    .def("collide", [](std::shared_ptr<collision::Point> &cc,
                       std::shared_ptr<collision::CollisionObject> &co) {
      return cc->collide(*co);
    })
    .def("center", [](collision::Point &point) {
      Eigen::Vector2d tmp = point.center();
      std::vector<double> pos;
      pos.push_back(tmp(0));
      pos.push_back(tmp(1));
      return py::array(2, pos.data());
    });


  py::class_<collision::RectangleAABB,
             std::shared_ptr<collision::RectangleAABB> >(
      m, "RectAABB", py::base<collision::Shape>())
    .def(py::init<double,double>())
    .def("__init__", [](collision::RectangleAABB &instance, double r_x,
                        double r_y, double x, double y) {
      new (&instance) collision::RectangleAABB(r_x, r_y, Eigen::Vector2d(x,y));
    })
    .def("collide", [](std::shared_ptr<collision::RectangleAABB> &cc,
                       std::shared_ptr<collision::CollisionObject> &co) {
      return cc->collide(*co);
    })
    .def("__str__", [](const std::shared_ptr<collision::RectangleAABB> &c) {
    	return "<collision::RectangleAABB r_x=" + std::to_string(c->r_x()) + 
    	  " r_y=" + std::to_string(c->r_y()) + ">";
    })
    .def("set_all", &collision::RectangleAABB::set_all)
    .def("min_x", [](const std::shared_ptr<collision::RectangleAABB> &c) {
      Eigen::Vector2d tmp = c->min();
      return tmp(0);
    })
    .def("min_y", [](const std::shared_ptr<collision::RectangleAABB> &c) {
      Eigen::Vector2d tmp = c->min();
      return tmp(1);
    })
    .def("max_x", [](const std::shared_ptr<collision::RectangleAABB> &c) {
      Eigen::Vector2d tmp = c->max();
      return tmp(0);
    })
    .def("max_y", [](const std::shared_ptr<collision::RectangleAABB> &c) {
      Eigen::Vector2d tmp = c->max();
      return tmp(1);
    })
    .def_property_readonly("r_x", &collision::RectangleAABB::r_x);


  py::class_<collision::RectangleOBB,
             std::shared_ptr<collision::RectangleOBB> >(
      m, "RectOBB", py::base<collision::Shape>())
    .def(py::init<double, double, double>())
    .def("__init__", [](collision::RectangleOBB &instance, double r_x,
                        double r_y, double angle, double x, double y) {
      new (&instance) collision::RectangleOBB(r_x, r_y, angle,
                                              Eigen::Vector2d(x,y));
    })
    .def("collide", [](std::shared_ptr<collision::RectangleOBB> &cc,
                       std::shared_ptr<collision::CollisionObject> &co) {
      return cc->collide(*co);
    })
    .def("set_center", [](collision::RectangleOBB &rect, double x, double y) {
      rect.set_center(Eigen::Vector2d(x,y));
    })
    .def("local_x_axis", [](collision::RectangleOBB &rect) {
      Eigen::Vector2d tmp = rect.local_x_axis();
      std::vector<double> pos;
      pos.push_back(tmp(0));
      pos.push_back(tmp(1));
      return py::array(2, pos.data());
    })
    .def("local_y_axis", [](collision::RectangleOBB &rect) {
      Eigen::Vector2d tmp = rect.local_y_axis();
      std::vector<double> pos;
      pos.push_back(tmp(0));
      pos.push_back(tmp(1));
      return py::array(2, pos.data());
    })
    .def("r_x", &collision::RectangleOBB::r_x)
    .def("r_y", &collision::RectangleOBB::r_y)
    .def("center", [](collision::RectangleOBB &rect) {
      Eigen::Vector2d tmp = rect.center();
      std::vector<double> pos;
      pos.push_back(tmp(0));
      pos.push_back(tmp(1));
      return py::array(2, pos.data());
    })
    .def("get_bounding_volume", &collision::RectangleOBB::getBoundingVolume)
    .def("__str__", [](collision::RectangleOBB &c) {
      return "<collision::RectangleOBB r_x=" + std::to_string(c.r_x()) + 
        " r_y=" + std::to_string(c.r_y()) + " center_x="
        + std::to_string(c.center_x()) + " center_y="
        + std::to_string(c.center_y()) + ">";
    });


  py::class_<collision::Triangle, std::shared_ptr<collision::Triangle> >(
      m, "Triangle", py::base<collision::Shape>())
    .def(py::init<>())
    .def("__init__", [](collision::Triangle &instance, double x1, double y1,
                        double x2, double y2, double x3, double y3) {
      new (&instance) collision::Triangle(Eigen::Vector2d(x1,y1),
                                          Eigen::Vector2d(x2,y2),
                                          Eigen::Vector2d(x3,y3));
    })
    .def("collide", [](std::shared_ptr<collision::Triangle> &cc,
                       std::shared_ptr<collision::CollisionObject> &co) {
      return cc->collide(*co);
    })
    .def("vertices", [](collision::Triangle &obj) {
      py::list v1;
      v1.append(py::cast(obj.v1()[0]));
      v1.append(py::cast(obj.v1()[1]));
      py::list v2;
      v2.append(py::cast(obj.v2()[0]));
      v2.append(py::cast(obj.v2()[1]));
      py::list v3;
      v3.append(py::cast(obj.v3()[0]));
      v3.append(py::cast(obj.v3()[1]));

      py::list ret_list;
      ret_list.append(v1);
      ret_list.append(v2);
      ret_list.append(v3);
      return ret_list;
    })
    .def("__str__", [](collision::Triangle &c) {
      return "<collision::Triangle v1=" + std::to_string(c.v1()[0]) + "/"
        + std::to_string(c.v1()[1]) + " v2=" + std::to_string(c.v2()[0])
        + "/" + std::to_string(c.v2()[1]) + " v3="
        + std::to_string(c.v3()[0]) + "/" + std::to_string(c.v3()[1])
        + ">";});

  py::class_<collision::Sphere, std::shared_ptr<collision::Sphere> >(
    m, "Circle", py::base<collision::Shape>())
  .def(py::init<double, double, double>())
  .def("collide", [](std::shared_ptr<collision::Sphere> &cc,
                     std::shared_ptr<collision::CollisionObject> &co) {
    return cc->collide(*co);
  })
  .def("r", &collision::Sphere::radius)
  .def("x", &collision::Sphere::get_x)
  .def("y", &collision::Sphere::get_y);


  py::class_<collision::BoundingVolume,
             std::shared_ptr<collision::BoundingVolume> >(
    m, "BoundingVolume", py::base<collision::CollisionObject>())
    .def("__init__", [](collision::BoundingVolume &instance, double x_min,
                        double x_max, double y_min, double y_max, int t_min,
                        int t_max) {
      std::array<double,2>x = {x_min,x_max};
      std::array<double,2>y = {y_min,y_max};
      std::array<int,2> t ={t_min,t_max};
      new (&instance) collision::BoundingVolume(x , y, t);
    })
        // .def("__init__", [](collision::BoundingVolume &instance, py::list boundingvolumes) {

        //     typedef std::shared_ptr<const collision::BoundingVolume> BoundingVolumeConstPtr;
        //     std::vector<BoundingVolumeConstPtr> bv_vector;
        //     for (const auto &item : boundingvolumes) {
        //         bv_vector.push_back(item.cast<BoundingVolumeConstPtr>());
        //     }

        //     new (&instance) collision::BoundingVolume(bv_vector);
        // })
//        .def("collide", [](std::shared_ptr<collision::BoundingVolume> &cc, std::shared_ptr<collision::CollisionObject> &co) {
//            return cc->collide(*co);
//        })
//        .def("__str__", [](const std::shared_ptr<collision::BoundingVolume> &c) {
//            std::ostringstream stream;
//            c->print(stream);
//            return "<collision::BoundingVolume\n" + stream.str() + ">";
//        })
    .def("get_children_bounding_volumes", [](
        const std::shared_ptr<collision::BoundingVolume> &cc) {
      auto boundingvolumes = cc->getChildrenBoundingVolumes();
      py::list bv_list;
      for (auto &o : boundingvolumes) {
        bv_list.append(py::cast(o));
      }
      return bv_list;
    })
    .def("get_contained_collision_objects", [](
        const std::shared_ptr<collision::BoundingVolume> &cc) {
      auto collisionobjects = cc->getChildrenCollisionObjects();
      py::list co_list;
      for (auto &o : collisionobjects) {
        co_list.append(py::cast(o));
      }
      return co_list;
    })
    .def("min_x", [](const std::shared_ptr<collision::BoundingVolume> &c) {
      std::array<double,2> tmp = c->x();
      return tmp[0];
    })
    .def("max_x", [](const std::shared_ptr<collision::BoundingVolume> &c) {
      std::array<double,2> tmp = c->x();
      return tmp[1];
    })
    .def("min_y", [](const std::shared_ptr<collision::BoundingVolume> &c) {
      std::array<double,2> tmp = c->y();
      return tmp[0];
    })
    .def("max_y", [](const std::shared_ptr<collision::BoundingVolume> &c) {
      std::array<double,2> tmp = c->y();
        return tmp[1];
    })
    .def("get_bounding_volume", &collision::BoundingVolume::getBoundingVolume);


  py::class_<collision::TimeVariantCollisionObject,
             std::shared_ptr<collision::TimeVariantCollisionObject> >(
      m, "TimeVariantCollisionObject", py::base<collision::CollisionObject>())
    .def(py::init<int>())
    .def("collide", [](
        std::shared_ptr<collision::TimeVariantCollisionObject> &cc,
        std::shared_ptr<collision::CollisionObject> &co) {
      return cc->collide(*co);
    })
    .def("append_obstacle", [](collision::TimeVariantCollisionObject &obj,
                               std::shared_ptr<collision::CollisionObject> co) {
            obj.appendObstacle(co);
    })
    .def("time_start_idx",
         &collision::TimeVariantCollisionObject::time_start_idx)
    .def("time_end_idx",  &collision::TimeVariantCollisionObject::time_end_idx)
    .def("get_obstacle_at_time",
         &collision::TimeVariantCollisionObject::getObstacleAtTime);


  py::class_<collision::ShapeGroup, std::shared_ptr<collision::ShapeGroup> >(
      m, "ShapeGroup", py::base<collision::CollisionObject>())
    .def(py::init<>())
    .def("collide", [](std::shared_ptr<collision::ShapeGroup> &cc,
                       std::shared_ptr<collision::CollisionObject> &co) {
      return cc->collide(*co);
    })
    .def("add_shape", [](collision::ShapeGroup &obj,
                         std::shared_ptr<collision::Shape> co) {
      obj.addToGroup(co);
    })
    .def("size", [](collision::ShapeGroup &obj) {
      auto unpacked = obj.unpack();
      return unpacked.size();
    })
    .def("unpack", [](collision::ShapeGroup &obj) {
      auto unpacked = obj.unpack();
      py::list ret_list;
      for (auto &i : unpacked) {
        ret_list.append(py::cast(i)); // BUG?
      }
            //ret_list.append(py::cast(unpacked[0]));
            //std::shared_ptr<const collision::Shape> ret;

            //if (idx>=0 && idx<unpacked.size()) {
            //    ret =  unpacked[idx];
            //}
            return ret_list;
        });


  py::class_<collision::Polygon, std::shared_ptr<collision::Polygon> >(
      m, "Polygon", py::base<collision::CollisionObject>())
    //.def("__init__", [](collision::Polygon &instance, py::list outer_boundary, 
    //                    py::list holes, py::list python_mesh_triangles) {
      .def("__init__", [](collision::Polygon &instance,
                          std::vector<std::array<double, 2> > outer_boundary,
                          py::list holes, py::list python_mesh_triangles) {
      std::vector<Eigen::Vector2d> vertices;
      std::vector<std::vector<Eigen::Vector2d> > hole_vertices;
      std::vector<collision::TriangleConstPtr> mesh_triangles;

      for (const auto &vertex : outer_boundary) {
        vertices.push_back(Eigen::Vector2d(vertex[0], vertex[1]));
      }
      //for (const auto &vertex : outer_boundary) {
      //for (const py::list &vertex : outer_boundary) {
        //vertices.push_back(Eigen::Vector2d(vertex[0], vertex[1]))
      //}

      for (const auto &triangle : python_mesh_triangles) {
        mesh_triangles.push_back(triangle.cast<collision::TriangleConstPtr>());
      }
      new (&instance) collision::Polygon(vertices, hole_vertices,
                                         mesh_triangles);
      })
    .def("collide", [](std::shared_ptr<collision::Polygon> &cc,
                       std::shared_ptr<collision::CollisionObject> &co) {
      return cc->collide(*co);
    })
    .def("get_triangle_mesh", [](collision::Polygon &obj) {
      auto triangles = obj.getTriangleMesh();
      py::list ret_list;
      for (collision::TriangleConstPtr &i : triangles) {
        ret_list.append(i);
      }
      return ret_list;
    })
    .def("get_vertices", [](collision::Polygon &obj) {
      py::list vertices;
      for (const auto &vertex : obj.getVertices()) {
        py::list point;
        point.append(vertex(0));
        point.append(vertex(1));
        vertices.append(point);
      }
      return vertices;
    })
    .def("__str__", [](collision::Polygon &c) {
      std::stringstream ss;
      ss << "<collision::Polygon vertices=";
      for (const auto &v : c.getVertices()) {
        ss << "(" << v(0) << "/" << v(1) << ") ";
      }
      ss << "\n";
      ss << ">";
      std::string s = ss.str();
      return ss.str();
    });


  py::class_<collision::Trajectory, std::shared_ptr<collision::Trajectory> >(
      m, "Trajectory", py::base<collision::TimeVariantCollisionObject>())
    .def("__init__", [](collision::Trajectory &instance, int t_start, 
                        py::array_t<double> positionlist,
                        py::array_t<double> orientationlist, double length,
                        double width) {

      std::vector<Eigen::Vector2d> positions;
      std::vector<double> orientations;

      // *************************************************************************************
      // Dieser Aufruf muss so durchgefuehrt werden, um Numpy Arrays auszulesen
      // Ein einfacher Aufruf ueber "for(const auto& element : list)" ist hier nicht moeglich!
      // *************************************************************************************
      py::buffer_info info_pos = positionlist.request();
      py::buffer_info info_ori = orientationlist.request();

      //info_pos liest alle Elemente der Reihe nach aus --> immer 2 zusammenfassen
      for (unsigned int idx = 0; idx < (info_pos.shape[0]) ; idx++) {
        Eigen::Vector2d temp {((double*)info_pos.ptr)[idx * 2],
                              ((double*)info_pos.ptr)[idx * 2 + 1]};
        positions.push_back(temp);
        orientations.push_back(((double*)info_ori.ptr)[idx]);
      }

      new (&instance) collision::Trajectory(t_start , positions, orientations,
                                            length, width);
    })
    .def("collide", [](std::shared_ptr<collision::Trajectory> &cc, 
                       std::shared_ptr<collision::CollisionObject> &co) {
      return cc->collide(*co);
    })
    .def("time_start_idx", &collision::Trajectory::time_start_idx)
    .def("time_end_idx",  &collision::Trajectory::time_end_idx)
    .def("get_obstacle_at_time",  &collision::Trajectory::getObstacleAtTime);
//        .def("get_bounding_volume", &collision::Trajectory::getBoundingVolume);


  py::class_<collision::CollisionChecker,
             std::shared_ptr<collision::CollisionChecker> >(m,
                                                            "CollisionChecker")
    .def(py::init<>())
    .def("number_of_obstacles", &collision::CollisionChecker::numberOfObstacles)
    .def("__str__", [](const std::shared_ptr<collision::CollisionChecker> &c) {
      std::ostringstream stream;
      c->print(stream);
      return "<collision::CollisionChecker\n" + stream.str() + ">";
    })
    .def("add_collision_object",
         &collision::CollisionChecker::addCollisionObject)
    .def("collide", [](const std::shared_ptr<collision::CollisionChecker> &cc, 
                       std::shared_ptr<collision::CollisionObject> &co) {
      return cc->collide(co);
    })
    .def("find_all_colliding_objects", [](
        const std::shared_ptr<collision::CollisionChecker> &cc, 
        std::shared_ptr<collision::CollisionObject> &co) {
      std::vector<collision::CollisionObjectConstPtr> obstacles;
      bool collides = cc->collide(co, obstacles);
      py::list ret_list;
      for (auto &i : obstacles) {
        ret_list.append(py::cast(i));
      }
      return ret_list;
    })
    .def("time_slice", &collision::CollisionChecker::timeSlice)
    .def("get_obstacles", [](
        const std::shared_ptr<collision::CollisionChecker> &cc) {
      auto obstacles = cc->getObstacles();
      py::list ret_list;
      for (auto &o : obstacles) {
        //ret_list.append(py::cast(std::const_pointer_cast<collision::CollisionObject>(o)));
        ret_list.append(py::cast(o));
      }
      return ret_list;
    });


  py::class_<collision::BVHIntersection,
             std::shared_ptr<collision::BVHIntersection> >(m, 
                                                           "BVHIntersection")
    .def(py::init<collision::BoundingVolumeConstPtr,
                  collision::BoundingVolumeConstPtr>())
        /*.def("__init__", [](collision::BVHIntersection &instance,
                            collision::BoundingVolumeConstPtr bvh_a,
                            collision::BoundingVolumeConstPtr bvh_b) {
            new (&instance) collision::BVHIntersection(bvh_a, bvh_b);
        });*/
    .def("find_all_candidates", [](
        const std::shared_ptr<collision::BVHIntersection> &bvh_intersection) {
      auto candidates = bvh_intersection->findAllCandidates();
      py::list ret_list;
      for (auto &c : candidates) {
        py::list pair;
        pair.append(c.first);
        pair.append(c.second);
        ret_list.append(pair);
      }
      return ret_list;
    })
    .def("reset", &collision::BVHIntersection::reset)
    .def("has_next_candidates", &collision::BVHIntersection::hasNextCandidates)
    .def("next_candidate", [](
        const std::shared_ptr<collision::BVHIntersection> &bvh_intersection) {
      auto candidates = bvh_intersection->nextCandidates();
      return py::make_tuple(candidates.first, candidates.second);
    });

  m.def("create_bvh_from_obstacle_list", [](py::list py_obstacle_list) {
    std::vector<collision::CollisionObjectConstPtr> collision_object_list;
    for (const auto &item : py_obstacle_list) {
      collision_object_list.push_back(
        item.cast<collision::CollisionObjectConstPtr>());
    }
    return collision::buildBoundingVolumeHierarchy(collision_object_list);
  }, "comment ");
    

}