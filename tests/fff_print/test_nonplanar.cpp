#include <catch2/catch_all.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/NonPlanar/S4Deformation.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <set>
#include <sstream>

#include "test_helpers.hpp"
#include "test_utils.hpp"

using namespace Slic3r::Test;
using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// A G1 endpoint of the G-code body (after the first layer change), with the layer it belongs to.
struct Move
{
    std::map<char, double> axes; // every axis word seen so far, carried between moves
    double                 e       = 0.;
    int                    layer   = 0;
    double                 layer_z = 0.; // the layer's height in the sliced space (its ;Z: comment)
    std::string            type;         // the feature, from the last ;TYPE: comment
    bool                   xy      = false; // the line moved in X or Y (an unretract does not)
};

std::vector<Move> body_moves(const std::string &gcode)
{
    std::vector<Move>      out;
    std::map<char, double> pos;
    int                    layer   = 0;
    double                 layer_z = 0.;
    std::string            type;
    std::istringstream     in(gcode);
    std::string            line;
    while (std::getline(in, line)) {
        if (line.rfind(";LAYER_CHANGE", 0) == 0)
            ++layer;
        else if (line.rfind(";Z:", 0) == 0)
            layer_z = std::stod(line.substr(3));
        else if (line.rfind(";TYPE:", 0) == 0)
            type = line.substr(6);
        const std::string code = line.substr(0, line.find(';'));
        std::istringstream words(code);
        std::string        cmd;
        if (! (words >> cmd) || (cmd != "G1" && cmd != "G0"))
            continue;
        Move        m;
        std::string w;
        while (words >> w)
            if (w.size() > 1 && std::isalpha(static_cast<unsigned char>(w[0]))) {
                const double v = std::stod(w.substr(1));
                if (w[0] == 'E')
                    m.e = v;
                else if (w[0] != 'F')
                    pos[w[0]] = v;
                m.xy |= w[0] == 'X' || w[0] == 'Y';
            }
        if (layer == 0)
            continue;
        m.axes    = pos;
        m.layer   = layer;
        m.layer_z = layer_z;
        m.type    = type;
        out.push_back(m);
    }
    return out;
}

double total_extrusion(const std::vector<Move> &moves)
{
    double e = 0.;
    for (const Move &m : moves)
        e += m.e;
    return e;
}

bool has_word(const std::vector<Move> &moves, char axis)
{
    return std::any_of(moves.begin(), moves.end(), [axis](const Move &m) { return m.axes.count(axis) > 0; });
}

double axis(const Move &m, char a) { return m.axes.count(a) ? m.axes.at(a) : 0.; }

// The largest Z range covered by the extrusion of one layer.
double max_layer_z_span(const std::vector<Move> &moves)
{
    std::map<int, std::pair<double, double>> span;
    for (const Move &m : moves)
        if (m.e > 0.) {
            const double z  = axis(m, 'Z');
            auto         it = span.emplace(m.layer, std::make_pair(z, z)).first;
            it->second.first  = std::min(it->second.first, z);
            it->second.second = std::max(it->second.second, z);
        }
    double out = 0.;
    for (const auto &[layer, s] : span)
        out = std::max(out, s.second - s.first);
    return out;
}

// An upside-down square frustum, 10 mm across at the bed and 40 mm across 15 mm up: every side
// overhangs 45 degrees from vertical, more than s4_max_overhang (30).
TriangleMesh inverted_frustum()
{
    return TriangleMesh({ { -5, -5, 0 }, { 5, -5, 0 }, { 5, 5, 0 }, { -5, 5, 0 }, { -20, -20, 15 }, { 20, -20, 15 }, { 20, 20, 15 }, { -20, 20, 15 } },
                        { { 0, 2, 1 }, { 0, 3, 2 }, { 4, 5, 6 }, { 4, 6, 7 }, { 0, 1, 5 }, { 0, 5, 4 },
                          { 1, 2, 6 }, { 1, 6, 5 }, { 2, 3, 7 }, { 2, 7, 6 }, { 3, 0, 4 }, { 3, 4, 7 } });
}

DynamicPrintConfig config_with(std::initializer_list<ConfigBase::SetDeserializeItem> items)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(items);
    return config;
}

DynamicPrintConfig s4_config()
{
    return config_with({ { "s4_enabled", true }, { "use_relative_e_distances", true }, { "layer_height", 0.3 },
                         { "initial_layer_print_height", 0.3 }, { "enable_support", false } });
}

} // namespace

TEST_CASE("S4 printing curves the layers of an overhanging part", "[NonPlanar]")
{
    const std::vector<Move> moves = body_moves(slice({ inverted_frustum() }, s4_config()));

    REQUIRE(total_extrusion(moves) > 0.);
    // Flat slices of the deformed part become layers that climb within themselves.
    CHECK(max_layer_z_span(moves) > 0.3);
    // Only the tip is written: no tilt axis without a polar printer.
    CHECK_FALSE(has_word(moves, 'B'));
    for (const Move &m : moves)
        if (m.e > 0.)
            REQUIRE(axis(m, 'Z') > 0.);
}

namespace {

// The lowest and highest extrusion of each layer.
std::map<int, std::pair<double, double>> layer_z_spans(const std::vector<Move> &moves)
{
    std::map<int, std::pair<double, double>> span;
    for (const Move &m : moves)
        if (m.e > 0.) {
            auto it = span.emplace(m.layer, std::make_pair(axis(m, 'Z'), axis(m, 'Z'))).first;
            it->second.first  = std::min(it->second.first, axis(m, 'Z'));
            it->second.second = std::max(it->second.second, axis(m, 'Z'));
        }
    return span;
}

} // namespace

TEST_CASE("S4 printing keeps the layers flat below the planar height", "[NonPlanar]")
{
    DynamicPrintConfig config = s4_config();
    config.set_deserialize_strict({ { "s4_planar_height", 6. } });
    const std::vector<Move> moves = body_moves(slice({ inverted_frustum() }, config));

    // The 20 layers of 0.3 mm up to 6 mm are flat, each at its height; above it the part still
    // bends.
    size_t flat = 0;
    for (const auto &[layer, s] : layer_z_spans(moves))
        if (s.second < 6. + 1e-3) {
            CHECK_THAT(s.first, WithinAbs(0.3 * layer, 1e-3));
            CHECK_THAT(s.second, WithinAbs(0.3 * layer, 1e-3));
            ++flat;
        }
    CHECK(flat == 20);
    CHECK(max_layer_z_span(moves) > 0.3);
}

TEST_CASE("S4 printing lays its first layer flat on the bed, and eases into the bend above it", "[NonPlanar]")
{
    // The frustum overhangs from the bed up, so the layers above its foot lift: the first layer
    // still lies flat at its height, and every layer above stays above it.
    const std::vector<Move> moves = body_moves(slice({ inverted_frustum() }, s4_config()));
    const auto              spans = layer_z_spans(moves);
    REQUIRE(spans.count(1) == 1);
    CHECK_THAT(spans.at(1).first, WithinAbs(0.3, 1e-3));
    CHECK_THAT(spans.at(1).second, WithinAbs(0.3, 1e-3));
    for (const Move &m : moves)
        if (m.e > 0. && m.layer > 1)
            REQUIRE(axis(m, 'Z') > 0.3);
    CHECK(max_layer_z_span(moves) > 0.3);
}

namespace {

// The steepest slope, in degrees, between consecutive extrusions of a layer.
double steepest(const std::vector<Move> &moves)
{
    double      out  = 0.;
    const Move *prev = nullptr;
    for (const Move &m : moves) {
        if (m.e > 0. && prev != nullptr && prev->layer == m.layer) {
            const double run = std::hypot(axis(m, 'X') - axis(*prev, 'X'), axis(m, 'Y') - axis(*prev, 'Y'));
            if (run > 0.2)
                out = std::max(out, std::atan2(std::abs(axis(m, 'Z') - axis(*prev, 'Z')), run) * 180. / PI);
        }
        prev = &m;
    }
    return out;
}

} // namespace

TEST_CASE("S4 layers lean no further than a nozzle that cannot tilt clears", "[NonPlanar]")
{
    // The frustum's 45 degree overhang turns its layers steeper than 12 degrees; a vertical nozzle
    // clearing only 8 degrees keeps them within that.
    DynamicPrintConfig config = s4_config();
    CHECK(steepest(body_moves(slice({ inverted_frustum() }, config))) > 12.);
    config.set_deserialize_strict({ { "nonplanar_nozzle_clearance_angle", 8. } });
    CHECK(steepest(body_moves(slice({ inverted_frustum() }, config))) < 8.);
}

TEST_CASE("A maximum rotation beyond the toolhead's reach is limited, and slicing says where to change it", "[NonPlanar]")
{
    // Maximum rotation 45 degrees on a vertical nozzle clearing 8: limited to 8, with a warning
    // naming the toolhead settings; within the reach, no warning.
    const bool beyond = GENERATE(true, false);
    DynamicPrintConfig config = s4_config();
    config.set_deserialize_strict({ { "nonplanar_nozzle_clearance_angle", 8. }, { "s4_max_rotation_near", beyond ? 45. : 8. },
                                    { "s4_max_rotation_far", beyond ? 45. : 8. } });
    Print print;
    Test::init_and_process_print({ inverted_frustum() }, print, config);
    const auto  state  = print.objects().front()->step_state_with_warnings(posSlice);
    const bool  warned = std::any_of(state.warnings.begin(), state.warnings.end(), [](const PrintStateBase::Warning &w) {
        return w.message.find("limited to 8.0") != std::string::npos && w.message.find("Nozzle clearance angle") != std::string::npos;
    });
    DYNAMIC_SECTION((beyond ? "beyond" : "within")) {
        CHECK(warned == beyond);
    }
}

namespace {

// A T standing on the bed, 10 mm deep: a post 10 mm wide and 10 mm high under a slab 30 mm wide and
// 2 mm thick, whose wings overhang 10 mm on either side, their undersides 10 mm up.
TriangleMesh t_shape()
{
    const std::vector<Vec2f> profile { { -5.f, 0.f }, { 5.f, 0.f }, { 5.f, 10.f }, { 15.f, 10.f },
                                       { 15.f, 12.f }, { -15.f, 12.f }, { -15.f, 10.f }, { -5.f, 10.f } }; // x, z
    indexed_triangle_set its;
    for (float y : { -5.f, 5.f })
        for (const Vec2f &p : profile)
            its.vertices.emplace_back(p.x(), y, p.y());
    const int n = int(profile.size());
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        its.indices.emplace_back(j, i, i + n);
        its.indices.emplace_back(j, i + n, j + n);
    }
    // The ends: the post, and the slab as a fan from a top corner over the four corners of its
    // underside (two of them where the post meets it).
    for (int off : { 0, n })
        for (Vec3i32 t : { Vec3i32(0, 1, 2), Vec3i32(0, 2, 7), Vec3i32(5, 6, 7), Vec3i32(5, 7, 2), Vec3i32(5, 2, 3), Vec3i32(5, 3, 4) }) {
            if (off > 0)
                std::swap(t[1], t[2]);
            its.indices.emplace_back(t[0] + off, t[1] + off, t[2] + off);
        }
    if (its_volume(its) < 0.f)
        its_flip_triangles(its);
    return TriangleMesh(its);
}

// Slices a T at (100, 100) and returns the G-code body; `paint` marks facets of it.
std::vector<Move> slice_t(DynamicPrintConfig config, const std::function<void(ModelVolume &)> &paint = {})
{
    config.set_deserialize_strict({ { "layer_change_gcode", "G92 E0" } });
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "t.stl";
    ModelVolume *volume = object->add_volume(t_shape());
    if (paint)
        paint(*volume);
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, config);
    REQUIRE(print.validate().string.empty());
    return body_moves(Test::gcode(print));
}

// The centre of the T's post, from its first layer (the T is symmetric about it).
Vec2d t_centre(const std::vector<Move> &moves)
{
    BoundingBoxf box;
    for (const Move &m : moves)
        if (m.e > 0. && m.xy && m.layer == 1 && m.type.find("Support") == std::string::npos && m.type.find("Brim") == std::string::npos &&
            m.type.find("Skirt") == std::string::npos)
            box.merge(Vec2d(axis(m, 'X'), axis(m, 'Y')));
    return box.center();
}

} // namespace

TEST_CASE("S4 support stands on the bed and stops the top gap below the real overhang", "[NonPlanar]")
{
    // Support sliced under the deformed wings of a T is printed in columns up to the real wings:
    // its first layer flat on the bed at 0.3 mm, its top 0.3 mm (the top gap) below their undersides
    // at 10 mm, and none of it inside the T. Each column's layers are in their order: where a layer
    // is printed, none printed before it is higher (by more than a bead's give). Its layers slope
    // where the part above does, so only a column a quarter of a millimetre across is level.
    DynamicPrintConfig config = s4_config();
    config.set_deserialize_strict({ { "enable_support", true }, { "support_type", "normal(auto)" },
                                    { "support_on_build_plate_only", true }, { "support_top_z_distance", 0.3 },
                                    { "support_bottom_z_distance", 0.3 } });
    const std::vector<Move> moves  = slice_t(config);
    const Vec2d             centre = t_centre(moves);
    std::map<std::pair<int, int>, std::vector<std::pair<int, double>>> columns; // (layer, z) in 0.25 mm cells
    double under_wings_top = 0.;
    size_t support = 0;
    for (const Move &m : moves) {
        if (m.e <= 0. || ! m.xy || m.type.find("Support") == std::string::npos)
            continue;
        ++support;
        const double dx = axis(m, 'X') - centre.x(), dy = axis(m, 'Y') - centre.y(), z = axis(m, 'Z');
        CHECK(z >= 0.3 - 1e-3);
        const bool in_post = std::abs(dx) < 4.9 && std::abs(dy) < 4.9 && z < 9.9;
        const bool in_slab = std::abs(dx) < 14.9 && std::abs(dy) < 4.9 && z > 10.1 && z < 11.9;
        CHECK_FALSE(in_post);
        CHECK_FALSE(in_slab);
        if (std::abs(dx) > 6. && std::abs(dx) < 14. && std::abs(dy) < 4.)
            under_wings_top = std::max(under_wings_top, z);
        columns[{ int(std::floor(4. * dx)), int(std::floor(4. * dy)) }].emplace_back(m.layer, z);
    }
    REQUIRE(support > 0);
    CHECK(under_wings_top <= 10. - 0.3 + 0.02);
    CHECK(under_wings_top >= 10. - 0.3 - 0.3 - 0.02);
    size_t out_of_order = 0;
    for (auto &[cell, points] : columns) {
        std::map<int, std::pair<double, double>> by_layer; // lowest and highest z in each layer
        for (const auto &[layer, z] : points) {
            auto it = by_layer.emplace(layer, std::make_pair(z, z)).first;
            it->second.first  = std::min(it->second.first, z);
            it->second.second = std::max(it->second.second, z);
        }
        double below = -1.;
        for (const auto &[layer, span] : by_layer) {
            if (span.first < below - 0.1)
                ++out_of_order;
            below = std::max(below, span.second);
        }
    }
    CHECK(out_of_order == 0);
}

TEST_CASE("S4 prints support where it is painted on the part", "[NonPlanar]")
{
    // Manual support painted on the underside of the T's +X wing only: printed under that wing,
    // as it is without S4, though the deformation moves the wing in the sliced space.
    auto paint = [](ModelVolume &volume) {
        // The volume's mesh is centred on its origin: the wings' undersides are 10 mm above its bottom.
        TriangleSelector           selector(volume.mesh());
        const indexed_triangle_set &its = volume.mesh().its;
        const BoundingBoxf3         box = volume.mesh().bounding_box();
        for (int i = 0; i < int(its.indices.size()); ++i) {
            const stl_vertex a = its.vertices[its.indices[i][0]], b = its.vertices[its.indices[i][1]], c = its.vertices[its.indices[i][2]];
            const Vec3f      normal = (b - a).cross(c - a);
            if (normal.z() < 0.f && std::abs(a.z() - (box.min.z() + 10.)) < 1e-3 && (a.x() + b.x() + c.x()) / 3. > box.center().x() + 5.)
                selector.set_facet(i, EnforcerBlockerType::ENFORCER);
        }
        volume.supported_facets.set(selector);
    };
    for (bool s4 : { false, true }) {
        DynamicPrintConfig config = s4_config();
        config.set_deserialize_strict({ { "s4_enabled", s4 }, { "enable_support", true }, { "support_type", "normal(manual)" },
                                        { "support_on_build_plate_only", true } });
        const std::vector<Move> moves  = slice_t(config, paint);
        const Vec2d             centre = t_centre(moves);
        size_t                  support = 0, under_other_wing = 0;
        for (const Move &m : moves)
            if (m.e > 0. && m.xy && m.type.find("Support") != std::string::npos) {
                ++support;
                if (axis(m, 'X') - centre.x() < 0.)
                    ++under_other_wing;
            }
        INFO("S4 " << s4);
        CHECK(support > 0);
        CHECK(under_other_wing == 0);
    }
}

TEST_CASE("S4 warns before slicing a part with nothing overhanging", "[NonPlanar]")
{
    // A cube's layers would be bent and bent back for nothing; a T's wings overhang.
    for (const bool t : { false, true }) {
        Model        model;
        ModelObject *object = model.add_object();
        object->name        = t ? "t.stl" : "cube.stl";
        object->add_volume(t ? t_shape() : TriangleMesh(its_make_cube(20., 20., 20.)));
        object->add_instance()->set_offset(Vec3d(100., 100., 0.));
        Print print;
        for (ModelObject *mo : model.objects)
            print.auto_assign_extruders(mo);
        DynamicPrintConfig config = s4_config();
        config.set_deserialize_strict({ { "layer_change_gcode", "G92 E0" } });
        print.apply(model, config);
        std::vector<StringObjectException> warnings;
        const StringObjectException        error = print.validate(&warnings);
        INFO(error.string);
        REQUIRE(error.string.empty());
        const bool warned = std::any_of(warnings.begin(), warnings.end(),
                                        [](const StringObjectException &w) { return w.opt_key == "s4_max_overhang"; });
        INFO(object->name);
        CHECK(warned == ! t);
    }
}

TEST_CASE("S4 meshes a large part finely only at its surface, and a small part never coarser than uniformly", "[NonPlanar]")
{
    // A 120 mm bar: 2 mm tetrahedra at the surface instead of the automatic 6 mm (1/20 of 120). A 20
    // mm cube: its automatic 1 mm caps the graded sizes, so it is no coarser than a uniform mesh.
    auto surface_edge = [](const TriangleMesh &part, bool graded) {
        Print              print;
        Model              model;
        DynamicPrintConfig config = s4_config();
        config.set_deserialize_strict({ { "s4_graded_mesh", graded }, { "s4_surface_cell_size", 2. }, { "s4_interior_cell_size", 5. },
                                        { "printable_area", "0x0,250x0,250x250,0x250" } });
        init_print({ part }, print, model, config);
        print.process();
        const NonPlanar::TetMesh &mesh  = print.objects().front()->s4_deformation()->mesh;
        double                   sum   = 0.;
        size_t                   count = 0;
        for (const auto &t : NonPlanar::s4_boundary_triangles(mesh, mesh.points))
            for (int k = 0; k < 3; ++k, ++count)
                sum += (mesh.points[t[k]] - mesh.points[t[(k + 1) % 3]]).norm();
        return sum / double(count);
    };
    const TriangleMesh bar(its_make_cube(120., 20., 20.));
    CHECK(surface_edge(bar, true) < 3.);
    CHECK(surface_edge(bar, false) > 4.5);
    const TriangleMesh cube(its_make_cube(20., 20., 20.));
    CHECK(surface_edge(cube, true) <= surface_edge(cube, false) * 1.1);
}

TEST_CASE("S4 printing requires relative extrusion", "[NonPlanar]")
{
    Print print;
    Model model;
    init_print({ inverted_frustum() }, print, model, config_with({ { "s4_enabled", true }, { "use_relative_e_distances", false } }));
    CHECK_FALSE(print.validate().string.empty());
}

TEST_CASE("Polar export drives the bed angle and radius instead of X and Y", "[NonPlanar]")
{
    // The bed turns about the centre of its printable area, (100, 100) here.
    const DynamicPrintConfig cartesian = config_with({ { "use_relative_e_distances", true }, { "printable_area", "0x0,200x0,200x200,0x200" } });
    DynamicPrintConfig       polar     = cartesian;
    polar.set_deserialize_strict({ { "polar_kinematics", true } });
    const std::vector<Move> ref = body_moves(slice({ TestMesh::L }, cartesian));
    const std::vector<Move> out = body_moves(slice({ TestMesh::L }, polar));

    CHECK(has_word(out, 'C'));
    CHECK_FALSE(has_word(out, 'Y'));
    // The same filament, split over more, shorter machine moves.
    CHECK_THAT(total_extrusion(out), WithinRel(total_extrusion(ref), 1e-4));
    CHECK(out.size() > ref.size());

    // Every extruding machine move puts the nozzle over the Cartesian print.
    BoundingBoxf printed;
    for (const Move &m : ref)
        if (m.e > 0.)
            printed.merge(Vec2d(axis(m, 'X'), axis(m, 'Y')));
    printed.offset(0.1);
    for (const Move &m : out)
        if (m.e > 0.) {
            const double angle = axis(m, 'C') * PI / 180.;
            const Vec2d  tip   = Vec2d(100., 100.) + axis(m, 'X') * Vec2d(std::cos(angle), std::sin(angle));
            REQUIRE(printed.contains(tip));
        }
}

TEST_CASE("Polar S4 export tilts the nozzle over curved layers", "[NonPlanar]")
{
    DynamicPrintConfig config = s4_config();
    config.set_deserialize_strict({ { "polar_kinematics", true }, { "polar_tilt_axis", true } });
    const std::vector<Move> moves = body_moves(slice({ inverted_frustum() }, config));

    REQUIRE(total_extrusion(moves) > 0.);
    double max_tilt = 0.;
    for (const Move &m : moves)
        max_tilt = std::max(max_tilt, std::abs(axis(m, 'B')));
    CHECK(max_tilt > 5.);
    CHECK(max_layer_z_span(moves) > 0.3);
}

TEST_CASE("The preview of a polar print knows each move's machine pose and machine G-code line", "[NonPlanar]")
{
    DynamicPrintConfig config = s4_config();
    config.set_deserialize_strict({ { "polar_kinematics", true }, { "polar_tilt_axis", true }, { "nonplanar_head_radius", 15. } });
    Print print;
    Model model;
    init_print({ inverted_frustum() }, print, model, config);
    print.set_status_silent();
    print.process();
    ScopedTemporaryFile  file(".gcode");
    GCodeProcessorResult result;
    print.export_gcode(file.string(), &result, nullptr);
    std::ifstream     in(file.string(), std::ios::binary);
    const std::string gcode((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    const GCodeProcessorResult::NonPlanarPreview &np = result.nonplanar;
    CHECK(np.toolhead);
    CHECK_THAT(np.head_radius, WithinAbs(15., 1e-6));
    REQUIRE(np.polar);
    REQUIRE(np.poses.size() == result.moves.size());
    // The line ends are the written machine G-code's.
    REQUIRE_FALSE(result.lines_ends.empty());
    CHECK(result.lines_ends.back() <= gcode.size());
    CHECK(gcode.size() - result.lines_ends.back() < 2);

    // Each extrusion's line is the machine move that ends it, at the recorded pose.
    auto word = [](const std::string &line, char letter) {
        const size_t at = line.find(std::string(" ") + letter);
        return at == std::string::npos ? std::nan("") : std::stod(line.substr(at + 2));
    };
    size_t checked = 0;
    float  max_tilt = 0.f;
    for (size_t i = 0; i < result.moves.size(); ++i) {
        const GCodeProcessorResult::NonPlanarPreview::MachinePose &pose = np.poses[i];
        if (result.moves[i].type != EMoveType::Extrude)
            continue;
        REQUIRE(pose.valid);
        const unsigned int id = result.moves[i].gcode_id;
        REQUIRE(id >= 1);
        REQUIRE(id <= result.lines_ends.size());
        const size_t      start = id == 1 ? 0 : result.lines_ends[id - 2];
        const std::string line  = gcode.substr(start, result.lines_ends[id - 1] - start);
        INFO(line);
        REQUIRE(line.rfind("G1 C", 0) == 0);
        CHECK_THAT(word(line, 'C'), WithinAbs(pose.angle, 1e-3));
        CHECK_THAT(word(line, 'X'), WithinAbs(pose.radius, 1e-3));
        CHECK_THAT(word(line, 'B'), WithinAbs(pose.tilt, 1e-3));
        max_tilt = std::max(max_tilt, std::abs(pose.tilt));
        ++checked;
    }
    CHECK(checked > 100);
    CHECK(max_tilt > 5.f);
}

namespace {

// Top of the layer holding `top` (0.3 mm layers): the print surface is printed up to it, and
// what is offset from the surface above it.
double surface_top(double top)
{
    return 0.3 + std::ceil((top - 0.3) / 0.3 - 1e-6) * 0.3;
}

// Upper half of a sphere standing on the bed.
TriangleMesh dome(double radius)
{
    indexed_triangle_set upper, lower;
    cut_mesh(its_make_sphere(radius, PI / 36.), 0.f, &upper, &lower, true);
    return TriangleMesh(upper);
}

} // namespace

TEST_CASE("A part is printed on a pillar, lifted off the bed", "[NonPlanar]")
{
    // A 10 x 10 x 4 mm block on a pillar 8 mm wide and 5 mm high: the pillar first, then the
    // block standing the gap above it, from 5.4 to 9.4 mm, its corners beyond the pillar in layers
    // wrapping round the pillar's rim.
    const double post_diameter = 8., post_height = 5., gap = 0.4, height = 4.;
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "block.stl";
    object->add_volume(TriangleMesh(its_make_cube(10., 10., height)));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));

    DynamicPrintConfig config = config_with({ { "s4_enabled", true }, { "s4_layer_shape", "offset" }, { "s4_surface_core", "pillar" },
                                              { "s4_surface_size", "custom" }, { "s4_surface_diameter", post_diameter },
                                              { "s4_surface_height", post_height },
                                              { "s4_surface_gap", gap }, { "use_relative_e_distances", true },
                                              { "layer_change_gcode", "G92 E0" }, { "skirt_loops", 0 }, { "brim_type", "no_brim" },
                                              { "layer_height", 0.3 }, { "initial_layer_print_height", 0.3 },
                                              { "enable_support", false } });
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, config);
    REQUIRE(print.validate().string.empty());
    const std::vector<Move> moves = body_moves(Test::gcode(print));

    // The post's first layer, on the bed, is as wide as the post.
    BoundingBoxf first;
    for (const Move &m : moves)
        if (m.e > 0. && m.layer == 1)
            first.merge(Vec2d(axis(m, 'X'), axis(m, 'Y')));
    CHECK(first.size().x() <= post_diameter);
    CHECK(first.size().x() >= post_diameter - 1.);
    const Vec2d centre = first.center();

    const double post_top = surface_top(post_height);
    size_t       post_points = 0, block_points = 0;
    double       post_reach = 0., block_low = 1e9, block_high = 0., block_reach = 0.;
    for (const Move &m : moves) {
        if (m.e <= 0.)
            continue;
        const Vec2d xy(axis(m, 'X'), axis(m, 'Y'));
        if (m.layer_z <= post_top + 1e-3) {
            ++post_points;
            post_reach = std::max(post_reach, (xy - centre).norm());
        } else {
            ++block_points;
            block_low   = std::min(block_low, axis(m, 'Z'));
            block_high  = std::max(block_high, axis(m, 'Z'));
            block_reach = std::max({ block_reach, std::abs(xy.x() - centre.x()), std::abs(xy.y() - centre.y()) });
        }
    }
    REQUIRE(post_points > 0);
    REQUIRE(block_points > 0);
    CHECK(post_reach <= 0.5 * post_diameter);
    CHECK(block_low >= post_height + gap);
    // Where the layers curve, a layer's last extrusion lies on its top, up to half a layer above
    // the block's flat top.
    CHECK(block_high <= post_height + gap + height + 0.5 * 0.3 + 0.01);
    CHECK(block_high >= post_height + gap + height - 0.35);
    CHECK(block_reach <= 5.05);
    CHECK(block_reach >= 4.5); // beyond the post
}

TEST_CASE("A generated pillar is printed as support, in its own layer height", "[NonPlanar]")
{
    // A 10 x 10 x 4 mm block on a pillar 12 mm wide and 6 mm high, printed as support in 0.3 mm
    // layers under the block's 0.15 mm ones: support and support interface only, its top layers
    // interface, and in between, a wall and the sparse support base, far less filament than the
    // layer printed solid.
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "block.stl";
    object->add_volume(TriangleMesh(its_make_cube(10., 10., 4.)));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    DynamicPrintConfig config = config_with({ { "s4_enabled", true }, { "s4_layer_shape", "offset" }, { "s4_surface_core", "pillar" },
                                              { "s4_surface_size", "custom" }, { "s4_surface_diameter", 12. },
                                              { "s4_surface_height", 6. }, { "s4_surface_layer_height", 0.3 },
                                              { "use_relative_e_distances", true }, { "layer_change_gcode", "G92 E0" },
                                              { "skirt_loops", 0 }, { "brim_type", "no_brim" }, { "layer_height", 0.15 },
                                              { "initial_layer_print_height", 0.3 }, { "enable_support", false } });
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, config);
    REQUIRE(print.validate().string.empty());
    const std::vector<Move> moves = body_moves(Test::gcode(print));
    const double            top   = print.objects().front()->s4_deformation()->surface_top;
    CHECK_THAT(top, WithinAbs(6., 1e-6)); // on the 0.3 mm grid
    double           interface_e = 0., middle_e = 0.; // middle: the pillar's layer at 3 mm
    std::set<double> heights;
    for (const Move &m : moves) {
        if (m.e <= 0. || ! m.xy || m.layer_z > top + 1e-3)
            continue;
        heights.insert(m.layer_z);
        CHECK(m.type.find("Support") != std::string::npos);
        if (m.type == "Support interface") {
            CHECK(m.layer_z > top - 3 * 0.3 - 1e-3);
            interface_e += m.e;
        }
        if (std::abs(m.layer_z - 3.) < 1e-3)
            middle_e += m.e;
    }
    CHECK(heights.size() == 20); // 0.3 to 6 mm
    CHECK(interface_e > 0.);
    const double filament_area = 0.25 * PI * std::pow(config.opt_float("filament_diameter", 0), 2);
    CHECK(middle_e * filament_area < 0.5 * 0.25 * PI * 12. * 12. * 0.3);
}

TEST_CASE("A part much wider than its pillar is high is printed at its full size", "[NonPlanar]")
{
    // A solid plate 30 mm wide and 1.5 mm thick on an automatic pillar 2 mm high (the printer does
    // not tilt): the layers over the pillar's flat top are flat, so the plate takes as much filament
    // as its volume.
    const double width = 30., thickness = 1.5;
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "plate.stl";
    object->add_volume(TriangleMesh(its_make_cube(width, width, thickness)));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    DynamicPrintConfig config = config_with({ { "s4_enabled", true }, { "s4_layer_shape", "offset" }, { "s4_surface_core", "pillar" },
                                              { "s4_surface_size", "auto" }, { "use_relative_e_distances", true },
                                              { "layer_change_gcode", "G92 E0" }, { "skirt_loops", 0 }, { "brim_type", "no_brim" },
                                              { "layer_height", 0.3 }, { "initial_layer_print_height", 0.3 },
                                              { "sparse_infill_density", "100%" }, { "enable_support", false } });
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, config);
    REQUIRE(print.validate().string.empty());
    const std::vector<Move> moves = body_moves(Test::gcode(print));

    const PrintObject::S4Deformation *s4 = print.objects().front()->s4_deformation();
    REQUIRE(s4 != nullptr);
    CHECK_THAT(bounding_box(s4->core).size().z(), WithinAbs(2., 1e-3));
    double plate_e = 0.;
    for (const Move &m : moves)
        if (m.layer_z > s4->surface_top + 1e-3)
            plate_e += m.e;
    const double filament_area = 0.25 * PI * std::pow(config.opt_float("filament_diameter", 0), 2);
    CHECK_THAT(plate_e * filament_area, WithinRel(width * width * thickness, 0.1));
}

TEST_CASE("A part much wider than its pillar is printed flat over it and on cones beyond it", "[NonPlanar]")
{
    // A plate 30 mm wide and 1.5 mm thick on a pillar 6 mm wide and 10 mm high, like a 3DBenchy on a
    // stick: its layers are flat over the pillar and descend at the 30 degree cone angle beyond the
    // rim, and the plate is printed where it is, its bottom the gap over the pillar. Layers at a
    // constant distance from the pillar leaned towards vertical out there, and bent the plate.
    const double width = 30., thickness = 1.5, post_height = 10., gap = 0.3;
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "plate.stl";
    object->add_volume(TriangleMesh(its_make_cube(width, width, thickness)));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    DynamicPrintConfig config = config_with({ { "s4_enabled", true }, { "s4_layer_shape", "offset" }, { "s4_surface_core", "pillar" },
                                              { "s4_surface_size", "custom" }, { "s4_surface_diameter", 6. },
                                              { "s4_surface_height", post_height }, { "s4_surface_gap", gap }, { "s4_cone_angle", 30. },
                                              { "use_relative_e_distances", true }, { "layer_change_gcode", "G92 E0" },
                                              { "skirt_loops", 0 }, { "brim_type", "no_brim" }, { "layer_height", 0.3 },
                                              { "initial_layer_print_height", 0.3 }, { "enable_support", false } });
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, config);
    REQUIRE(print.validate().string.empty());
    const std::vector<Move> moves = body_moves(Test::gcode(print));

    std::vector<Move> plate;
    for (const Move &m : moves)
        if (m.layer_z > surface_top(post_height) + 1e-3)
            plate.push_back(m);
    REQUIRE(! plate.empty());
    BoundingBoxf3 box;
    for (const Move &m : plate)
        if (m.e > 0. && m.xy)
            box.merge(Vec3d(axis(m, 'X'), axis(m, 'Y'), axis(m, 'Z')));
    CHECK(steepest(plate) <= 30. + 2.);
    CHECK(box.min.z() >= post_height + gap - 0.05);
    CHECK(box.max.z() <= post_height + gap + thickness + 0.2);
    CHECK(box.size().x() >= width - 1.);
    CHECK(box.size().x() <= width + 0.1);
}

TEST_CASE("An automatic pillar is as high as the toolhead needs to lean under the part", "[NonPlanar]")
{
    // Leaning 60 degrees, the head's rim 20 mm out, 15 mm up the nozzle, drops 20 sin 60 - 15 cos 60
    // below the tip: the part must stand that high, on a pillar as wide as its base.
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "block.stl";
    object->add_volume(TriangleMesh(its_make_cube(10., 10., 4.)));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    DynamicPrintConfig config = config_with({ { "s4_enabled", true }, { "s4_layer_shape", "offset" }, { "s4_surface_core", "pillar" },
                                              { "s4_surface_size", "auto" }, { "use_relative_e_distances", true },
                                              { "layer_change_gcode", "G92 E0" }, { "printable_area", "0x0,200x0,200x200,0x200" },
                                              { "polar_kinematics", true }, { "polar_tilt_axis", true }, { "polar_tilt_min", -45 },
                                              { "polar_tilt_max", 60 }, { "nonplanar_head_radius", 20 }, { "nonplanar_nozzle_length", 15 },
                                              { "nonplanar_nozzle_tip_diameter", 0.8 }, { "nonplanar_nozzle_clearance_angle", 50 },
                                              { "enable_support", false } });
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, config);
    REQUIRE(print.validate().string.empty());
    Test::gcode(print);

    const PrintObject::S4Deformation *s4 = print.objects().front()->s4_deformation();
    REQUIRE(s4 != nullptr);
    const BoundingBoxf3 post = bounding_box(s4->core);
    CHECK_THAT(post.size().z(), WithinAbs(20. * std::sin(PI / 3.) - 15. * std::cos(PI / 3.), 1e-3));
    CHECK_THAT(post.size().x(), WithinAbs(std::sqrt(50.) * 2., 0.05)); // the base's corners
}

TEST_CASE("A wider toolhead re-slices a part on an automatic pillar onto a higher one", "[NonPlanar]")
{
    // As above, the head's rim 20 mm out stands the pillar 20 sin 60 - 15 cos 60 high; changing it
    // to 30 mm on the same print must slice again, onto a pillar 30 sin 60 - 15 cos 60 high.
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "block.stl";
    object->add_volume(TriangleMesh(its_make_cube(10., 10., 4.)));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    DynamicPrintConfig config = config_with({ { "s4_enabled", true }, { "s4_layer_shape", "offset" }, { "s4_surface_core", "pillar" },
                                              { "s4_surface_size", "auto" }, { "use_relative_e_distances", true },
                                              { "layer_change_gcode", "G92 E0" }, { "printable_area", "0x0,200x0,200x200,0x200" },
                                              { "polar_kinematics", true }, { "polar_tilt_axis", true }, { "polar_tilt_min", -45 },
                                              { "polar_tilt_max", 60 }, { "nonplanar_head_radius", 20 }, { "nonplanar_nozzle_length", 15 },
                                              { "nonplanar_nozzle_tip_diameter", 0.8 }, { "nonplanar_nozzle_clearance_angle", 50 },
                                              { "enable_support", false } });
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, config);
    Test::gcode(print);
    auto pillar_height = [&print]() { return bounding_box(print.objects().front()->s4_deformation()->core).size().z(); };
    CHECK_THAT(pillar_height(), WithinAbs(20. * std::sin(PI / 3.) - 15. * std::cos(PI / 3.), 1e-3));

    config.set_deserialize_strict({ { "nonplanar_head_radius", 30 } });
    print.apply(model, config);
    Test::gcode(print);
    CHECK_THAT(pillar_height(), WithinAbs(30. * std::sin(PI / 3.) - 15. * std::cos(PI / 3.), 1e-3));
}

TEST_CASE("A dome is a hemisphere of its diameter, the part on its top", "[NonPlanar]")
{
    // An 8 mm dome is 4 mm high; the block's flat base stands the gap above its top.
    const double diameter = 8., gap = 0.4;
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "block.stl";
    object->add_volume(TriangleMesh(its_make_cube(10., 10., 4.)));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    DynamicPrintConfig config = config_with({ { "s4_enabled", true }, { "s4_layer_shape", "offset" }, { "s4_surface_core", "dome" },
                                              { "s4_surface_size", "custom" }, { "s4_surface_diameter", diameter },
                                              { "s4_surface_gap", gap }, { "use_relative_e_distances", true },
                                              { "layer_change_gcode", "G92 E0" }, { "skirt_loops", 0 }, { "brim_type", "no_brim" },
                                              { "layer_height", 0.3 }, { "initial_layer_print_height", 0.3 },
                                              { "enable_support", false } });
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, config);
    REQUIRE(print.validate().string.empty());
    const std::vector<Move> moves = body_moves(Test::gcode(print));

    const BoundingBoxf3 dome = bounding_box(print.objects().front()->s4_deformation()->core);
    CHECK_THAT(dome.size().z(), WithinAbs(0.5 * diameter, 1e-3));
    CHECK_THAT(dome.size().x(), WithinAbs(diameter, 1e-3));
    double block_low = 1e9;
    for (const Move &m : moves)
        if (m.e > 0. && m.layer_z > surface_top(0.5 * diameter) + 1e-3)
            block_low = std::min(block_low, axis(m, 'Z'));
    CHECK(block_low >= 0.5 * diameter + gap);
}

namespace {

// Closed surface of revolution about Z of a profile in (r, z) that starts and ends on the axis.
TriangleMesh revolve(const std::vector<Vec2d> &profile, int segments = 96)
{
    indexed_triangle_set its;
    std::vector<int>     ring_start;
    for (const Vec2d &p : profile) {
        ring_start.push_back(int(its.vertices.size()));
        const int n = p.x() <= 0. ? 1 : segments;
        for (int k = 0; k < n; ++k) {
            const double a = 2. * PI * k / segments;
            its.vertices.emplace_back(float(p.x() * std::cos(a)), float(p.x() * std::sin(a)), float(p.y()));
        }
    }
    auto at = [&](size_t i, int k) { return profile[i].x() <= 0. ? ring_start[i] : ring_start[i] + k % segments; };
    for (size_t i = 0; i + 1 < profile.size(); ++i)
        for (int k = 0; k < segments; ++k) {
            const int a = at(i, k), b = at(i, k + 1), c = at(i + 1, k), d = at(i + 1, k + 1);
            if (a != b)
                its.indices.emplace_back(a, b, c);
            if (c != d)
                its.indices.emplace_back(b, d, c);
        }
    return TriangleMesh(its);
}

// A dome shell from inner radius `inner` to `outer`, open to the bed.
TriangleMesh hollow_dome(double inner, double outer)
{
    std::vector<Vec2d> profile;
    for (int k = 0; k <= 24; ++k) { // inner arc, top to bed
        const double a = 0.5 * PI * k / 24;
        profile.emplace_back(inner * std::sin(a), inner * std::cos(a));
    }
    for (int k = 24; k >= 0; --k) { // outer arc, bed to top
        const double a = 0.5 * PI * k / 24;
        profile.emplace_back(outer * std::sin(a), outer * std::cos(a));
    }
    return revolve(profile);
}

} // namespace

TEST_CASE("A generated sphere core puts the first layer on the part's inner surface", "[NonPlanar]")
{
    // Only the shell is modelled: the slicer fits a core into it, the gap short of its inner
    // surface, prints it first, and the shell's first layer lies on its inner surface.
    const double inner = 20., outer = 25., gap = 0.4;
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "shell.stl";
    object->add_volume(hollow_dome(inner, outer));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));

    DynamicPrintConfig config = config_with({ { "s4_enabled", true }, { "s4_layer_shape", "offset" }, { "s4_surface_core", "sphere" },
                                              { "s4_surface_gap", gap }, { "use_relative_e_distances", true },
                                              { "skirt_loops", 0 }, { "brim_type", "no_brim" },
                                              { "layer_change_gcode", "G92 E0" }, { "layer_height", 0.3 },
                                              { "initial_layer_print_height", 0.3 }, { "enable_support", false } });
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, config);
    const StringObjectException invalid = print.validate();
    INFO(invalid.string);
    REQUIRE(invalid.string.empty());
    const std::vector<Move> moves = body_moves(Test::gcode(print));

    BoundingBoxf first;
    for (const Move &m : moves)
        if (m.e > 0. && m.layer == 1)
            first.merge(Vec2d(axis(m, 'X'), axis(m, 'Y')));
    const Vec3d centre(first.center().x(), first.center().y(), 0.);

    const double core_radius = inner - gap;
    const double core_top    = surface_top(core_radius);
    size_t       core_points = 0, shell_points = 0;
    double       core_max = 0., shell_min = 1e9;
    for (const Move &m : moves) {
        if (m.e <= 0. || m.layer == 1) // only the core prints on the bed
            continue;
        const double d = (Vec3d(axis(m, 'X'), axis(m, 'Y'), axis(m, 'Z')) - centre).norm();
        if (m.layer_z <= core_top + 1e-3) {
            ++core_points;
            core_max = std::max(core_max, d);
        } else {
            ++shell_points;
            shell_min = std::min(shell_min, d);
        }
    }
    REQUIRE(core_points > 0);
    REQUIRE(shell_points > 0);
    CHECK(core_max <= core_radius + 0.35);
    // The shell's first layer is its modelled inner surface.
    CHECK(shell_min >= inner - 0.05);
}

namespace {

// A ring standing on the bed about the Z axis.
TriangleMesh ring(double inner, double outer, double height)
{
    indexed_triangle_set its = revolve({ { inner, 0. }, { outer, 0. }, { outer, height }, { inner, height }, { inner, 0. } }).its;
    its_merge_vertices(its);
    return TriangleMesh(its);
}

// Paints the volume's faces for which `painted(centre, normal)` holds as print surface.
template<class Painted> void paint_print_surface(ModelVolume &volume, Painted painted)
{
    const indexed_triangle_set &its = volume.mesh().its;
    TriangleSelector            selector(volume.mesh());
    for (size_t i = 0; i < its.indices.size(); ++i) {
        const auto &t = its.indices[i];
        const Vec3d a = its.vertices[t[0]].cast<double>(), b = its.vertices[t[1]].cast<double>(), c = its.vertices[t[2]].cast<double>();
        if (painted((a + b + c) / 3., (b - a).cross(c - a).normalized()))
            selector.set_facet(int(i), EnforcerBlockerType::ENFORCER);
    }
    volume.print_surface_facets.set(selector);
}

DynamicPrintConfig painted_config(std::initializer_list<ConfigBase::SetDeserializeItem> items)
{
    DynamicPrintConfig config = config_with({ { "s4_enabled", true }, { "s4_layer_shape", "offset" }, { "s4_surface_core", "painted" },
                                              { "use_relative_e_distances", true }, { "layer_change_gcode", "G92 E0" },
                                              { "skirt_loops", 0 }, { "brim_type", "no_brim" }, { "layer_height", 0.3 },
                                              { "initial_layer_print_height", 0.3 }, { "enable_support", false },
                                              { "printable_area", "0x0,200x0,200x200,0x200" } });
    config.set_deserialize_strict(items);
    return config;
}

bool is_support(const Move &m) { return m.type.find("Support") != std::string::npos; }

} // namespace

TEST_CASE("Layers offset from painted faces start on them, the print surface under them the gap away", "[NonPlanar]")
{
    // A dome shell 20 mm inside and 26 mm outside, its inside painted: the print surface fills it,
    // the gap short of it, as support, down to the bed under the shell lifted 2 mm (automatic, on a
    // printer that does not tilt); the shell is printed from its inside out.
    const double inner = 20., outer = 26., gap = 0.4;
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "shell.stl";
    ModelVolume *shell  = object->add_volume(hollow_dome(inner, outer));
    paint_print_surface(*shell, [](const Vec3d &c, const Vec3d &n) { return n.dot(c) < -0.1 * c.norm(); });
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, painted_config({ { "s4_surface_gap", gap } }));
    const StringObjectException invalid = print.validate();
    INFO(invalid.string);
    REQUIRE(invalid.string.empty());
    const std::vector<Move> moves = body_moves(Test::gcode(print));
    const auto             *s4    = print.objects().front()->s4_deformation();
    CHECK_THAT(s4->lift, WithinAbs(2., 1e-9));
    const Vec3d centre(100., 100., 2.);
    size_t      core_points = 0, shell_points = 0;
    double      core_max = 0., shell_min = 1e9, shell_max = 0.;
    for (const Move &m : moves) {
        if (m.e <= 0. || ! m.xy)
            continue;
        const double d = (Vec3d(axis(m, 'X'), axis(m, 'Y'), axis(m, 'Z')) - centre).norm();
        if (m.layer_z <= s4->surface_top + 1e-3) {
            CHECK(is_support(m));
            ++core_points;
            if (axis(m, 'Z') > 2.) // over the column the shell is lifted by
                core_max = std::max(core_max, d);
        } else {
            CHECK(! is_support(m));
            ++shell_points;
            shell_min = std::min(shell_min, d);
            shell_max = std::max(shell_max, d);
        }
    }
    REQUIRE(core_points > 0);
    REQUIRE(shell_points > 0);
    CHECK(core_max <= inner - gap + 0.35); // the top layer's cap, printed at the layer's top
    CHECK(shell_min >= inner - 0.05);      // its first layer on its inside
    CHECK(shell_max <= outer + 0.5);
}

TEST_CASE("A part standing on painted faces is lifted onto the print surface under them", "[NonPlanar]")
{
    // A 10 x 10 x 4 mm block with its base painted: lifted 2 mm (automatic, on a printer that does
    // not tilt) or as high as set, onto a print surface under its base, the gap below it.
    const bool   custom = GENERATE(false, true);
    const double lift = custom ? 5. : 2., gap = 0.4;
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "block.stl";
    ModelVolume *block  = object->add_volume(TriangleMesh(its_make_cube(10., 10., 4.)));
    paint_print_surface(*block, [](const Vec3d &, const Vec3d &n) { return n.z() < -0.9; });
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, painted_config({ { "s4_surface_gap", gap }, { "s4_surface_size", custom ? "custom" : "auto" },
                                        { "s4_surface_height", 5. } }));
    REQUIRE(print.validate().string.empty());
    const std::vector<Move> moves = body_moves(Test::gcode(print));
    CHECK_THAT(print.objects().front()->s4_deformation()->lift, WithinAbs(lift, 1e-6));
    BoundingBoxf3 core, part;
    for (const Move &m : moves)
        if (m.e > 0. && m.xy)
            (is_support(m) ? core : part).merge(Vec3d(axis(m, 'X'), axis(m, 'Y'), axis(m, 'Z')));
    REQUIRE(core.defined);
    REQUIRE(part.defined);
    CHECK(core.max.z() <= lift - gap + 0.35);
    CHECK(part.min.z() >= lift - 0.05);
    CHECK(part.max.z() <= lift + 4. + 0.35);
    CHECK(core.size().x() <= 10.05);
    CHECK(core.size().y() <= 10.05);
}

TEST_CASE("A ring is printed round its painted inside, lifted for the toolhead", "[NonPlanar]")
{
    // A ring 30 mm inside and 34 mm outside, 8 mm high, round the rotation axis, its inside
    // painted, laid out round the axis: a column of print surface up to its top, the gap inside its
    // inside, and the ring in cylindrical layers from its inside out, lifted 2 mm.
    const double inner = 30., outer = 34., height = 8., gap = 0.4, lift = 2.;
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "ring.stl";
    ModelVolume *band   = object->add_volume(ring(inner, outer, height));
    paint_print_surface(*band, [](const Vec3d &c, const Vec3d &n) {
        const Vec3d r(c.x(), c.y(), 0.);
        return r.norm() > 0. && n.dot(r.normalized()) < -0.5;
    });
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, painted_config({ { "s4_surface_gap", gap }, { "s4_surface_projection", "axis" } }));
    const StringObjectException invalid = print.validate();
    INFO(invalid.string);
    REQUIRE(invalid.string.empty());
    const std::vector<Move> moves = body_moves(Test::gcode(print));
    CHECK_THAT(print.objects().front()->s4_deformation()->lift, WithinAbs(lift, 1e-6));
    double                        core_max = 0., ring_min = 1e9, ring_max = 0., ring_low = 1e9, core_top = 0.;
    std::map<int, std::pair<double, double>> layer_radii;
    for (const Move &m : moves) {
        if (m.e <= 0. || ! m.xy)
            continue;
        const double r = std::hypot(axis(m, 'X') - 100., axis(m, 'Y') - 100.);
        if (is_support(m)) {
            core_max = std::max(core_max, r);
            core_top = std::max(core_top, axis(m, 'Z'));
        } else {
            ring_min = std::min(ring_min, r);
            ring_max = std::max(ring_max, r);
            ring_low = std::min(ring_low, axis(m, 'Z'));
            auto [it, added] = layer_radii.try_emplace(m.layer, r, r);
            it->second = { std::min(it->second.first, r), std::max(it->second.second, r) };
        }
    }
    REQUIRE(core_max > 0.);
    REQUIRE(ring_max > 0.);
    CHECK(core_max <= inner - gap + 0.1);
    CHECK(core_top <= lift + height + 0.35);
    CHECK(ring_min >= inner - 0.1); // its first layer on its inside
    CHECK(ring_max <= outer + 0.5);
    CHECK(ring_low >= lift - 0.05);
    // Each of its layers a cylinder: the same distance from the axis all round and all the way up.
    for (const auto &[layer, radii] : layer_radii) {
        INFO("layer " << layer << ": " << radii.first << " to " << radii.second);
        CHECK(radii.second - radii.first < 0.6);
    }
}

TEST_CASE("Painted faces that do not face the print surface are refused", "[NonPlanar]")
{
    // A block painted on top gives nothing to lay out from above; a ring painted inside half way
    // round, nothing to lay out round the axis; a part painted nowhere is refused before slicing.
    const int kind = GENERATE(0, 1, 2);
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "part.stl";
    ModelVolume *part   = object->add_volume(kind == 1 ? ring(30., 34., 8.) : TriangleMesh(its_make_cube(10., 10., 4.)));
    if (kind == 0)
        paint_print_surface(*part, [](const Vec3d &, const Vec3d &n) { return n.z() > 0.9; });
    else if (kind == 1)
        paint_print_surface(*part, [](const Vec3d &c, const Vec3d &n) {
            const Vec3d r(c.x(), c.y(), 0.);
            return c.y() > 0. && n.dot(r.normalized()) < -0.5;
        });
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, painted_config({ { "s4_surface_projection", kind == 1 ? "axis" : "above" } }));
    if (kind == 2) {
        CHECK_THAT(print.validate().string, Catch::Matchers::ContainsSubstring("painted as print surface"));
        return;
    }
    REQUIRE(print.validate().string.empty());
    CHECK_THROWS_WITH(Test::gcode(print), Catch::Matchers::ContainsSubstring(kind == 0 ? "faces down" : "all the way round"));
}

TEST_CASE("A print surface part of an older project becomes painted faces", "[NonPlanar]")
{
    // A dome shell over a 20 mm dome set as print surface: the part is removed and the shell's
    // faces within reach of it, its inside, painted.
    Model        model;
    ModelObject *object = model.add_object();
    ModelVolume *shell  = object->add_volume(hollow_dome(20.4, 26.));
    object->add_volume(dome(20.))->config.set_key_value("s4_print_surface", new ConfigOptionBool(true));
    object->add_instance();
    CHECK(convert_legacy_print_surface_parts(*object));
    REQUIRE(object->volumes.size() == 1);
    REQUIRE(object->volumes.front() == shell);
    indexed_triangle_set painted = shell->print_surface_facets.get_facets(*shell, EnforcerBlockerType::ENFORCER);
    REQUIRE(! painted.indices.empty());
    // From the domes' centre, the middle of the shell's base.
    const BoundingBoxf3 box = shell->mesh().bounding_box();
    const Vec3f         centre(float(box.center().x()), float(box.center().y()), float(box.min.z()));
    for (const auto &t : painted.indices)
        CHECK(((painted.vertices[t[0]] + painted.vertices[t[1]] + painted.vertices[t[2]]) / 3.f - centre).norm() < 21.5f);
    CHECK(! convert_legacy_print_surface_parts(*object));
}

TEST_CASE("A placed print surface is printed onto, not printed, and kept clear of", "[NonPlanar]")
{
    // A 20 mm dome set as placed print surface under a dome shell 20.4 mm inside: nothing is printed
    // for the dome, not even as support; the shell is printed from its inside out, from the
    // first layer on, not lifted; and no move, travel included, goes into the dome.
    const double placed_radius = 20., inner = 20.4, outer = 26.;
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "shell.stl";
    object->add_volume(hollow_dome(inner, outer));
    object->add_volume(dome(placed_radius))->config.set_key_value("s4_placed_surface", new ConfigOptionBool(true));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, painted_config({ { "s4_surface_gap", 0.4 } }));
    const StringObjectException invalid = print.validate();
    INFO(invalid.string);
    REQUIRE(invalid.string.empty());
    const std::vector<Move> moves = body_moves(Test::gcode(print));
    CHECK_THAT(print.objects().front()->s4_deformation()->lift, WithinAbs(0., 1e-9));
    // No empty layers under the shell: the G-code preview needs every layer to hold moves.
    CHECK(! print.objects().front()->layers().front()->lslices.empty());
    const Vec3d centre(100., 100., 0.);
    size_t      printed = 0;
    double      nearest_print = 1e9, nearest_move = 1e9;
    for (const Move &m : moves) {
        if (! m.xy)
            continue;
        const double d = (Vec3d(axis(m, 'X'), axis(m, 'Y'), axis(m, 'Z')) - centre).norm();
        nearest_move   = std::min(nearest_move, d);
        if (m.e > 0.) {
            CHECK(! is_support(m));
            ++printed;
            nearest_print = std::min(nearest_print, d);
        }
    }
    REQUIRE(printed > 0);
    CHECK(nearest_print >= inner - 0.05);
    CHECK(nearest_move >= placed_radius);
}

TEST_CASE("A toolhead clearing only a few degrees can be set", "[NonPlanar]")
{
    // A Voron Stealthburner-like head: 2.2 degrees from the nozzle's edge out to 65.5 mm across.
    const ConfigOptionDef *def = print_config_def.get("nonplanar_nozzle_clearance_angle");
    REQUIRE(def != nullptr);
    CHECK(def->min <= 2.2);
}

TEST_CASE("A part short of resting on its placed print surface is refused", "[NonPlanar]")
{
    // The shell is 2 mm from the placed dome, more than the 0.4 mm gap: its first layer would be in mid-air.
    Model        model;
    ModelObject *object = model.add_object();
    object->add_volume(hollow_dome(20.4, 26.));
    object->add_volume(dome(18.4))->config.set_key_value("s4_placed_surface", new ConfigOptionBool(true));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, painted_config({ { "s4_surface_gap", 0.4 } }));
    REQUIRE(print.validate().string.empty());
    CHECK_THROWS_WITH(Test::gcode(print), Catch::Matchers::ContainsSubstring("does not rest on its placed print surface"));
}

TEST_CASE("A skirt or brim is refused with a placed print surface, not dropped", "[NonPlanar]")
{
    // Round the part's first layer, on the placed surface, either could run into it: the user turns them off.
    const bool skirt = GENERATE(true, false);
    Model        model;
    ModelObject *object = model.add_object();
    object->add_volume(hollow_dome(20.4, 26.));
    object->add_volume(dome(20.))->config.set_key_value("s4_placed_surface", new ConfigOptionBool(true));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    if (skirt)
        print.apply(model, painted_config({ { "skirt_loops", 1 }, { "skirt_height", 1 } }));
    else
        print.apply(model, painted_config({ { "brim_type", "outer_only" }, { "brim_width", 5. } }));
    DYNAMIC_SECTION((skirt ? "skirt" : "brim")) {
        CHECK_THAT(print.validate().string, Catch::Matchers::ContainsSubstring(skirt ? "Skirt loops" : "Brim type"));
    }
}

TEST_CASE("Placed print surface parts need layers offset from a print surface", "[NonPlanar]")
{
    Model        model;
    ModelObject *object = model.add_object();
    object->add_volume(hollow_dome(20.4, 26.));
    object->add_volume(dome(20.))->config.set_key_value("s4_placed_surface", new ConfigOptionBool(true));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, painted_config({ { "s4_layer_shape", "optimized" } }));
    CHECK_THAT(print.validate().string, Catch::Matchers::ContainsSubstring("Placed print surface parts need"));
}

TEST_CASE("A polar printer is chosen by its structure, and older presets load as one", "[NonPlanar]")
{
    // The structure Polar turns the conversion on.
    DynamicPrintConfig chosen;
    chosen.set_deserialize_strict("printer_structure", "polar");
    chosen.set_deserialize_strict("polar_kinematics", "0");
    chosen.handle_legacy_composite();
    CHECK(chosen.opt_bool("polar_kinematics"));
    // A preset from before the structure had a polar value has only the switch.
    DynamicPrintConfig older;
    older.set_deserialize_strict("printer_structure", "undefine");
    older.set_deserialize_strict("polar_kinematics", "1");
    older.handle_legacy_composite();
    CHECK(older.option<ConfigOptionEnum<PrinterStructure>>("printer_structure")->value == psPolar);
    // Any other printer stays as it is.
    DynamicPrintConfig other;
    other.set_deserialize_strict("printer_structure", "corexy");
    other.set_deserialize_strict("polar_kinematics", "0");
    other.handle_legacy_composite();
    CHECK_FALSE(other.opt_bool("polar_kinematics"));
    CHECK(other.option<ConfigOptionEnum<PrinterStructure>>("printer_structure")->value == psCoreXY);
}
