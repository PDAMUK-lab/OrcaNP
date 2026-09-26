#include "NonPlanarExport.hpp"

#include "../BoundingBox.hpp"
#include "../Print.hpp"

namespace Slic3r {
namespace NonPlanarExport {

namespace {

// The printer's rotation axis, in G-code coordinates: the centre of the printable area.
Vec2d rotation_axis(const PrintConfig &config) { return BoundingBoxf(config.printable_area.values).center(); }

} // namespace

bool has_s4(const Print &print)
{
    for (const PrintObject *object : print.objects())
        if (object->s4_deformation() != nullptr)
            return true;
    return false;
}

std::unique_ptr<NonPlanar::S4Mapper> s4_mapper(const Print &print)
{
    // All deformed objects in one mesh, moved from each object's slicing frame into G-code
    // coordinates: XY by the instance shift less the plate origin, Z by the Z offset and a raft.
    NonPlanar::TetMesh           mesh;
    std::vector<Eigen::Vector3d> deformed;
    for (const PrintObject *object : print.objects()) {
        const PrintObject::S4Deformation *s4 = object->s4_deformation();
        if (s4 == nullptr)
            continue;
        const Vec2d           xy = unscale(object->instances().front().shift) - print.get_plate_origin().head<2>();
        const Eigen::Vector3d offset(xy.x(), xy.y(), print.config().z_offset.value + object->slicing_parameters().object_print_z_min);
        const int             base = int(mesh.points.size());
        for (size_t v = 0; v < s4->mesh.points.size(); ++v) {
            mesh.points.push_back(s4->mesh.points[v] + offset);
            deformed.push_back(s4->deformed[v] + offset);
        }
        for (const std::array<int, 4> &t : s4->mesh.tets)
            mesh.tets.push_back({ t[0] + base, t[1] + base, t[2] + base, t[3] + base });
    }
    if (mesh.tets.empty())
        return nullptr;
    return std::make_unique<NonPlanar::S4Mapper>(mesh, deformed, rotation_axis(print.config()));
}

NonPlanar::S4GCodeConfig s4_gcode_config(const PrintConfig &config)
{
    NonPlanar::S4GCodeConfig cfg;
    cfg.z_floor   = config.z_offset.value;
    cfg.z_hop     = std::max(0.2, config.z_hop.get_at(0));
    cfg.emit_tilt = config.polar_kinematics.value && config.polar_tilt_axis.value;
    if (config.polar_axis_names.value.size() == 3)
        cfg.tilt_axis = config.polar_axis_names.value[2];
    return cfg;
}

NonPlanar::PolarKinematicsConfig polar_config(const PrintConfig &config)
{
    NonPlanar::PolarKinematicsConfig cfg;
    cfg.center            = rotation_axis(config);
    cfg.angle_sign        = config.polar_reverse_rotation.value ? -1. : 1.;
    cfg.min_radius        = config.polar_min_radius.value;
    cfg.max_angle_step    = config.polar_angle_step.value;
    cfg.max_angular_speed = config.polar_max_rotation_speed.value * 60.;
    cfg.max_tilt_speed    = config.polar_max_tilt_speed.value * 60.;
    cfg.inverse_time_feed = config.polar_inverse_time_feed.value;
    cfg.has_tilt_axis     = config.polar_tilt_axis.value;
    cfg.tilt_pivot_length = config.polar_tilt_pivot_length.value;
    if (config.polar_axis_names.value.size() == 3) {
        cfg.angle_axis  = config.polar_axis_names.value[0];
        cfg.radius_axis = config.polar_axis_names.value[1];
        cfg.tilt_axis   = config.polar_axis_names.value[2];
    }
    return cfg;
}

} // namespace NonPlanarExport
} // namespace Slic3r
