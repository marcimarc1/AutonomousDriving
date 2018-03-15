#include "trajectory.h"
#include "rectangle_obb.h"
#include "shapegroup.h"
#include "shape.h"

namespace collision {

Trajectory::Trajectory(int time_start_idx, std::vector<Eigen::Vector2d> positions, std::vector<double> orientations,
                       double length, double width, double max_interp_distance)
    :TimeVariantCollisionObject(time_start_idx)
{
    positions_ = positions;
    orientations_ = orientations;

    //  reduce angles >2pi so only one half turn is performed at maximum
    for (int i=0; i<orientations.size(); i++) {
        orientations_[i] = fmod((fmod(orientations_[i], (2 * M_PI)) + 2 * M_PI), (2 * M_PI));
        if (orientations_[i] > M_PI) {orientations_[i] -= 2*M_PI;}
    }

    // interpolate Trajectory between time steps
    for (int pose_ct=0; pose_ct<orientations.size()-1; pose_ct++) {
        Eigen::Vector2d tangent = positions_[pose_ct+1] - positions_[pose_ct];
        double dist = tangent.norm();

        // always turn in the shortest direction
        double angle = orientations_[pose_ct] - orientations_[pose_ct+1];
        if (angle > M_PI) {angle -= 2 * M_PI;}
        else if (angle < -M_PI) {angle += 2 * M_PI;}

        //build up one shape group per time step
        ShapeGroup* sg = new ShapeGroup();
        for (int subpose_ct=0; subpose_ct < int(dist/max_interp_distance); subpose_ct++) {
            double r = subpose_ct * max_interp_distance / dist;
            RectangleOBB* obb = new RectangleOBB(0.5*length,0.5*width, orientations_[pose_ct] - r * angle, positions_[pose_ct] + r * tangent);
            std::shared_ptr<RectangleOBB> obb_ptr = std::shared_ptr<RectangleOBB>(obb);
            sg->addToGroup(obb_ptr);
        }
        // include one OBB for the last time step
        RectangleOBB* obb_end = new RectangleOBB(0.5*length,0.5*width, orientations_[pose_ct+1], positions_[pose_ct+1]);
        std::shared_ptr<RectangleOBB> obbend_ptr = std::shared_ptr<RectangleOBB>(obb_end);
        sg->addToGroup(obbend_ptr);

        std::shared_ptr<ShapeGroup> sg_ptr = std::shared_ptr<ShapeGroup>(sg);
        appendObstacle(sg_ptr);
    }
}

void Trajectory::print(std::ostringstream &stream) const {
    stream << "Trajectory, time " << time_start_idx_ << "-" <<time_end_idx_ << std::endl;
    for (int i=0; i<collision_object_at_time_.size(); i++) {
        stream << "  " << i+time_start_idx_ << ":";
        collision_object_at_time_[i]->print(stream);
    }
}

BoundingVolumeConstPtr Trajectory::getBoundingVolume() const {
    std::array<double,2> x = {INFINITY,-INFINITY};
    std::array<double,2> y = {INFINITY,-INFINITY};
    std::array<int,2> t = {time_start_idx_, time_end_idx_};

    for (auto &object : collision_object_at_time_) {
        BoundingVolumeConstPtr temp = object->getBoundingVolume();
        std::array<double,2> x_temp = temp->x();
        std::array<double,2> y_temp = temp->y();
        if (x_temp[0] < x[0]) { x[0] = x_temp[0]; }
        if (x_temp[1] > x[1]) { x[1] = x_temp[1]; }
        if (y_temp[0] < y[0]) { y[0] = y_temp[0]; }
        if (y_temp[1] > y[1]) { y[1] = y_temp[1]; }
    }

    // ****************************************************************************
    // Hier kann eine Optimierung zum Aufspalten der Trajektorie eingefuegt werden
    // ****************************************************************************

    BoundingVolume* bv = new BoundingVolume(x,y,t,shared_from_this());
    std::shared_ptr<BoundingVolume> bv_ptr = std::shared_ptr<BoundingVolume>(bv);
    return bv_ptr;
}

}
