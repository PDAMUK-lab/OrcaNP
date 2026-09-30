#include "NonPlanarExport.hpp"

#include "../BoundingBox.hpp"
#include "../Exception.hpp"
#include "../Geometry.hpp"
#include "../I18N.hpp"
#include "../format.hpp"
#include "../Print.hpp"
#include "GCodeProcessor.hpp"

#include <boost/nowide/fstream.hpp>

#include <optional>

namespace Slic3r {
namespace NonPlanarExport {

namespace {

// The printer's rotation axis, in G-code coordinates: the centre of the printable area.
Vec2d rotation_axis(const PrintConfig &config) { return BoundingBoxf(config.printable_area.values).center(); }

} // namespace

int marker_id(const Print &print, const PrintObject &object)
{
    int id = 0;
    for (const PrintObject *o : print.objects()) {
        if (o == &object)
            break;
        ++id;
    }
    return id;
}

static bool tilts(const ConfigBase &printer_config)
{
    return printer_config.option<ConfigOptionBool>("polar_kinematics")->value && printer_config.option<ConfigOptionBool>("polar_tilt_axis")->value;
}

double s4_lean_limit(const ConfigBase &printer_config)
{
    auto value = [&printer_config](const char *key) { return printer_config.option<ConfigOptionFloat>(key)->value; };
    return tilts(printer_config) ? std::min(-value("polar_tilt_min"), value("polar_tilt_max")) : value("nonplanar_nozzle_clearance_angle");
}

std::string s4_lean_limit_where(const ConfigBase &printer_config)
{
    return tilts(printer_config) ?
               _u8L("To change it, change how far the nozzle tilts: Printer settings > Basic information > Polar kinematics "
                    "(Minimum tilt, Maximum tilt).") :
               _u8L("To change it, change the shape of the toolhead's boundary: Printer settings > Basic information > "
                    "Non-planar toolhead (Nozzle clearance angle, Nozzle length, Toolhead radius).");
}

bool has_s4(const Print &print)
{
    for (const PrintObject *object : print.objects())
        if (object->s4_deformation() != nullptr)
            return true;
    return false;
}

std::unique_ptr<S4Mappers> s4_mappers(const Print &print)
{
    // Each deformed object is moved from its slicing frame into G-code coordinates: XY by the
    // instance shift less the plate origin, Z by the Z offset and a raft.
    auto out = std::make_unique<S4Mappers>();
    for (const PrintObject *object : print.objects()) {
        const PrintObject::S4Deformation *s4 = object->s4_deformation();
        if (s4 == nullptr)
            continue;
        // G-code positions are also less the offset of the extruder printing them (GCode::point_to_gcode()),
        // which must be the same for all of the object's extruders.
        std::optional<Vec2d> nozzle;
        for (unsigned int filament : object->object_extruders()) {
            const Vec2d o = print.config().extruder_offset.get_at(print.get_extruder_id(filament));
            if (nozzle && (o - *nozzle).norm() > EPSILON)
                throw Slic3r::SlicingError(format(_u8L("%1% is printed non-planar by extruders at different offsets (Extruder offset): "
                                                       "print it with one extruder."),
                                                  object->model_object()->name));
            nozzle = o;
        }
        const Vec2d xy = unscale(object->instances().front().shift) - print.get_plate_origin().head<2>() - nozzle.value_or(Vec2d::Zero());
        const Eigen::Vector3d offset(xy.x(), xy.y(), print.config().z_offset.value + object->slicing_parameters().object_print_z_min);
        NonPlanar::TetMesh           mesh = s4->mesh;
        std::vector<Eigen::Vector3d> deformed = s4->deformed;
        for (Eigen::Vector3d &p : mesh.points)
            p += offset;
        for (Eigen::Vector3d &p : deformed)
            p += offset;
        out->storage.push_back(std::make_unique<NonPlanar::S4Mapper>(mesh, deformed, rotation_axis(print.config())));
        NonPlanar::S4ObjectMapping &m = out->set.objects[marker_id(print, *object)];
        m.mapper           = out->storage.back().get();
        // G-code heights carry three decimals: the surface's top layer must not read as above it.
        m.identity_below_z = s4->surface_top + offset.z() + 1e-3;
        m.flat_top_z       = s4->flat_top + offset.z();
        m.blend_top_z      = m.flat_top_z + s4->blend;
        m.keep_min_x       = s4->keep_min_x + offset.x();
        m.keep_max_x       = s4->keep_max_x + offset.x();
        // A placed print surface is not printed but is in the nozzle's way: its surface, sampled
        // about half a millimetre apart.
        for (const stl_triangle_vertex_indices &t : s4->placed.indices) {
            const Eigen::Vector3d a = s4->placed.vertices[t[0]].cast<double>(), b = s4->placed.vertices[t[1]].cast<double>(),
                                  c = s4->placed.vertices[t[2]].cast<double>();
            const int n = std::clamp(int(std::ceil(std::max({ (b - a).norm(), (c - b).norm(), (a - c).norm() }) / 0.5)), 1, 200);
            for (int i = 0; i <= n; ++i)
                for (int j = 0; i + j <= n; ++j)
                    m.obstacles.push_back(a + (b - a) * (double(i) / n) + (c - a) * (double(j) / n) + offset);
        }
        // Support is mapped in columns between the part's surfaces. Layers offset from a print
        // surface have overhangs that say nothing about the part's, so theirs is left as it was.
        if (! object->support_layers().empty() && object->config().s4_layer_shape.value != S4LayerShape::Offset) {
            indexed_triangle_set surface = s4->surface;
            its_translate(surface, offset.cast<float>());
            out->surfaces.push_back(std::make_unique<NonPlanar::S4SupportSurface>(surface));
            m.support_surface    = out->surfaces.back().get();
            m.support_top_gap    = object->config().support_top_z_distance.value;
            m.support_bottom_gap = object->config().support_bottom_z_distance.value;
        }
    }
    if (out->storage.empty())
        return nullptr;
    return out;
}

NonPlanar::S4GCodeConfig s4_gcode_config(const PrintConfig &config)
{
    NonPlanar::S4GCodeConfig cfg;
    cfg.z_floor   = config.z_offset.value;
    cfg.z_hop     = std::max(0.2, config.z_hop.get_at(0));
    cfg.emit_tilt = config.polar_kinematics.value && config.polar_tilt_axis.value;
    if (config.polar_axis_names.value.size() == 3)
        cfg.tilt_axis = config.polar_axis_names.value[2];
    // The polar conversion may put the nozzle on either side of the rotation axis, which flips
    // the sign of the tilt, so only the travel both sides share is safe.
    cfg.tilt_threshold = Geometry::deg2rad(config.polar_tilt_threshold.value);
    if (config.polar_signed_radius.value)
        cfg.max_tilt = Geometry::deg2rad(std::min(-config.polar_tilt_min.value, config.polar_tilt_max.value));
    else {
        // The radius never turns negative, so the part's outward lean is the machine's.
        cfg.min_tilt_toward = Geometry::deg2rad(config.polar_tilt_min.value);
        cfg.max_tilt_away   = Geometry::deg2rad(config.polar_tilt_max.value);
    }
    cfg.clearance_check   = config.nonplanar_clearance_check.value;
    // The clearance angle rises from the tip's face; the cone's half-angle is from the nozzle axis.
    cfg.nozzle_cone_angle = Geometry::deg2rad(90. - config.nonplanar_nozzle_clearance_angle.value);
    cfg.nozzle_tip_radius = 0.5 * config.nonplanar_nozzle_tip_diameter.value;
    cfg.nozzle_length     = config.nonplanar_nozzle_length.value;
    cfg.head_radius       = config.nonplanar_head_radius.value;
    return cfg;
}

NonPlanar::PolarKinematicsConfig polar_config(const PrintConfig &config)
{
    NonPlanar::PolarKinematicsConfig cfg;
    cfg.center            = rotation_axis(config);
    cfg.angle_sign        = config.polar_reverse_rotation.value ? -1. : 1.;
    cfg.min_radius        = config.polar_min_radius.value;
    cfg.radius_offset     = config.polar_radius_offset.value;
    cfg.radius_scale      = config.polar_radius_scale.value / 100.;
    cfg.max_angle_step    = config.polar_angle_step.value;
    cfg.max_angular_speed = config.polar_max_rotation_speed.value * 60.;
    cfg.max_tilt_speed    = config.polar_max_tilt_speed.value * 60.;
    cfg.inverse_time_feed = config.polar_inverse_time_feed.value;
    cfg.has_tilt_axis     = config.polar_tilt_axis.value;
    cfg.tilt_pivot_length = config.polar_tilt_pivot_length.value;
    cfg.min_tilt          = config.polar_tilt_min.value;
    cfg.max_tilt          = config.polar_tilt_max.value;
    cfg.tilt_sign         = config.polar_reverse_tilt.value ? -1. : 1.;
    cfg.signed_radius     = config.polar_signed_radius.value;
    cfg.min_travel_radius = config.polar_radius_min.value;
    cfg.max_travel_radius = config.polar_radius_max.value;
    if (config.polar_axis_names.value.size() == 3) {
        cfg.angle_axis  = config.polar_axis_names.value[0];
        cfg.radius_axis = config.polar_axis_names.value[1];
        cfg.tilt_axis   = config.polar_axis_names.value[2];
    }
    return cfg;
}

void set_toolhead_preview(GCodeProcessorResult &result, const PrintConfig &config)
{
    GCodeProcessorResult::NonPlanarPreview &np = result.nonplanar;
    np.toolhead          = true;
    np.nozzle_tip_radius = float(0.5 * config.nonplanar_nozzle_tip_diameter.value);
    // The clearance angle rises from the tip's face; the cone's half-angle is from the nozzle axis.
    np.nozzle_cone_angle = float(90. - config.nonplanar_nozzle_clearance_angle.value);
    np.nozzle_length     = float(config.nonplanar_nozzle_length.value);
    np.head_radius       = float(config.nonplanar_head_radius.value);
}

void map_preview_to_polar(GCodeProcessorResult &result, const NonPlanar::PolarGCodeConverter &converter, const PrintConfig &config,
                          const std::string &polar_path)
{
    const NonPlanar::PolarKinematicsConfig  pc = polar_config(config);
    GCodeProcessorResult::NonPlanarPreview &np = result.nonplanar;
    np.polar         = true;
    np.rotation_axis = pc.center.cast<float>();
    np.angle_sign    = float(pc.angle_sign);
    np.tilt_sign     = float(pc.tilt_sign);

    // Line ids count from 1, over the lines the converter read.
    const std::vector<NonPlanar::PolarGCodeConverter::LineRecord> &records = converter.line_records();
    np.poses.assign(result.moves.size(), {});
    for (size_t i = 0; i < result.moves.size(); ++i) {
        unsigned int &id = result.moves[i].gcode_id;
        if (id == 0 || id > records.size())
            continue;
        const NonPlanar::PolarGCodeConverter::LineRecord &r = records[id - 1];
        if (r.posed)
            np.poses[i] = { true, r.angle, r.radius, r.z, r.tilt };
        id = r.out_lines;
    }

    // Each line's end is the position after its newline.
    result.lines_ends.clear();
    boost::nowide::ifstream in(polar_path, std::ios::binary);
    std::vector<char>       chunk(1 << 16);
    size_t                  offset = 0;
    while (in) {
        in.read(chunk.data(), std::streamsize(chunk.size()));
        const size_t n = size_t(in.gcount());
        for (size_t i = 0; i < n; ++i)
            if (chunk[i] == '\n')
                result.lines_ends.push_back(offset + i + 1);
        offset += n;
    }
}

} // namespace NonPlanarExport
} // namespace Slic3r
