#include <catch2/catch_all.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>

#include "test_helpers.hpp"

using namespace Slic3r::Test;
using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// A G1 endpoint of the G-code body (after the first layer change), with the layer it belongs to.
struct Move
{
    std::map<char, double> axes; // every axis word seen so far, carried between moves
    double                 e     = 0.;
    int                    layer = 0;
};

std::vector<Move> body_moves(const std::string &gcode)
{
    std::vector<Move>      out;
    std::map<char, double> pos;
    int                    layer = 0;
    std::istringstream     in(gcode);
    std::string            line;
    while (std::getline(in, line)) {
        if (line.rfind(";LAYER_CHANGE", 0) == 0)
            ++layer;
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
            }
        if (layer == 0)
            continue;
        m.axes  = pos;
        m.layer = layer;
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

double axis(const Move &m, char a) { return m.axes.count(a) ? m.axes.at(a) : 0.; }

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
        REQUIRE(axis(m, 'Z') > 0.);
}

TEST_CASE("S4 printing keeps the layers flat below the planar height", "[NonPlanar]")
{
    DynamicPrintConfig config = s4_config();
    config.set_deserialize_strict({ { "s4_planar_height", 6. } });
    const std::vector<Move> moves = body_moves(slice({ inverted_frustum() }, config));

    // Layers wholly below 6 mm are flat; above it the part still bends.
    std::map<int, std::pair<double, double>> span;
    for (const Move &m : moves)
        if (m.e > 0.) {
            auto it = span.emplace(m.layer, std::make_pair(axis(m, 'Z'), axis(m, 'Z'))).first;
            it->second.first  = std::min(it->second.first, axis(m, 'Z'));
            it->second.second = std::max(it->second.second, axis(m, 'Z'));
        }
    size_t flat = 0;
    for (const auto &[layer, s] : span)
        if (s.second < 5.5) {
            CHECK_THAT(s.second - s.first, WithinAbs(0., 1e-3));
            ++flat;
        }
    CHECK(flat >= 15); // 0.3 mm layers
    CHECK(max_layer_z_span(moves) > 0.3);
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

namespace {

// Upper half of a sphere standing on the bed.
TriangleMesh dome(double radius)
{
    indexed_triangle_set upper, lower;
    cut_mesh(its_make_sphere(radius, PI / 36.), 0.f, &upper, &lower, true);
    return TriangleMesh(upper);
}

} // namespace

TEST_CASE("Layers offset from a print surface part keep the surface gap all around it", "[NonPlanar]")
{
    // A 20 mm dome printed flat as the print surface, and over it a 26 mm dome: what is left of
    // it is a shell from the gap out to 26 mm, printed after the whole core.
    const double core_radius = 20., shell_radius = 26., gap = 0.4;
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = "dome.stl";
    ModelVolume *core   = object->add_volume(dome(core_radius));
    core->config.set_key_value("s4_print_surface", new ConfigOptionBool(true));
    object->add_volume(dome(shell_radius));
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));

    DynamicPrintConfig config = config_with({ { "s4_enabled", true }, { "s4_layer_shape", "offset" }, { "s4_surface_gap", gap },
                                              { "use_relative_e_distances", true }, { "layer_height", 0.3 },
                                              { "initial_layer_print_height", 0.3 }, { "enable_support", false } });
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, config);
    REQUIRE(print.validate().string.empty());
    const std::vector<Move> moves = body_moves(Test::gcode(print));

    // The dome's centre, on the bed, from the core's first layer.
    BoundingBoxf first;
    for (const Move &m : moves)
        if (m.e > 0. && m.layer == 1)
            first.merge(Vec2d(axis(m, 'X'), axis(m, 'Y')));
    const Vec3d centre(first.center().x(), first.center().y(), 0.);

    size_t core_points = 0, shell_points = 0, last_core = 0, first_shell = moves.size();
    double shell_min = 1e9, shell_max = 0.;
    for (size_t i = 0; i < moves.size(); ++i) {
        const Move &m = moves[i];
        if (m.e <= 0.)
            continue;
        const double d = (Vec3d(axis(m, 'X'), axis(m, 'Y'), axis(m, 'Z')) - centre).norm();
        if (d <= core_radius + 0.1) {
            ++core_points;
            last_core = i;
        } else {
            ++shell_points;
            first_shell = std::min(first_shell, i);
            shell_min   = std::min(shell_min, d);
            shell_max   = std::max(shell_max, d);
        }
    }
    REQUIRE(core_points > 0);
    REQUIRE(shell_points > 0);
    // Nothing is printed within the gap, and the shell stays inside its dome.
    CHECK(shell_min >= core_radius + gap - 0.05);
    CHECK(shell_max <= shell_radius + 0.3);
    // The whole core comes first.
    CHECK(last_core < first_shell);
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
                its.indices.emplace_back(a, c, b);
            if (c != d)
                its.indices.emplace_back(b, c, d);
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
                                              { "layer_height", 0.3 }, { "initial_layer_print_height", 0.3 },
                                              { "enable_support", false } });
    Print print;
    for (ModelObject *mo : model.objects)
        print.auto_assign_extruders(mo);
    print.apply(model, config);
    REQUIRE(print.validate().string.empty());
    const std::vector<Move> moves = body_moves(Test::gcode(print));

    BoundingBoxf first;
    for (const Move &m : moves)
        if (m.e > 0. && m.layer == 1)
            first.merge(Vec2d(axis(m, 'X'), axis(m, 'Y')));
    const Vec3d centre(first.center().x(), first.center().y(), 0.);

    size_t core_points = 0, shell_points = 0, last_core = 0, first_shell = moves.size();
    double shell_min = 1e9;
    for (size_t i = 0; i < moves.size(); ++i) {
        const Move &m = moves[i];
        if (m.e <= 0.)
            continue;
        const double d = (Vec3d(axis(m, 'X'), axis(m, 'Y'), axis(m, 'Z')) - centre).norm();
        if (d <= inner - gap + 0.1) {
            ++core_points;
            last_core = i;
        } else {
            ++shell_points;
            first_shell = std::min(first_shell, i);
            shell_min   = std::min(shell_min, d);
        }
    }
    REQUIRE(core_points > 0);
    REQUIRE(shell_points > 0);
    CHECK(shell_min >= inner - 0.05);
    CHECK(last_core < first_shell);
}

