#include <catch2/catch_all.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/CalibrationPrints.hpp"

#include <cmath>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::CalibrationPrints;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

BoundingBoxf3 bounds(const indexed_triangle_set &its)
{
    BoundingBoxf3 box;
    for (const Vec3f &v : its.vertices)
        box.merge(v.cast<double>());
    return box;
}

// Open edges once coincident vertices are one, as when the mesh is saved and loaded again.
int open_edges(indexed_triangle_set its)
{
    its_merge_vertices(its);
    return its_num_open_edges(its);
}

// A square's corners after printing on a machine whose Y axis leans `skew` degrees toward +X and
// which prints everything `scale` times its size.
struct Printed
{
    Vec2d a, b, c, d;
};
Printed print_square(double side, double skew, double scale)
{
    const double k = std::tan(skew * M_PI / 180.);
    auto         p = [&](double x, double y) { return Vec2d(scale * (x + k * y), scale * y); };
    return { p(0., 0.), p(side, 0.), p(side, side), p(0., side) };
}

} // namespace

TEST_CASE("The test prints are closed surfaces", "[CalibrationPrints]")
{
    CHECK(open_edges(square_frames(SquareFrames())) == 0);
    CHECK(open_edges(holes_and_pegs(hole_peg_diameters())) == 0);
    CHECK(open_edges(elephant_foot_block()) == 0);
    CHECK(open_edges(overhang_samples(overhang_angles(), 150.)) == 0);
    CHECK(open_edges(support_gap_sample("0.20")) == 0);
    CHECK(open_edges(surface_gap_sample("0.20")) == 0);
}

TEST_CASE("The square frames' bosses stand where the fit measures them from", "[CalibrationPrints]")
{
    const SquareFrames f;
    const indexed_triangle_set its = square_frames(f);
    // Above the flat frame's bars and below the upright frames' first boss tops, only the flat
    // frame's bosses reach out this far.
    double min_x = 0., max_y = 0.;
    for (const Vec3f &v : its.vertices)
        if (v.z() > f.height + 1e-3 && std::abs(v.x()) > 0.5 * f.side - 1.) {
            min_x = std::min(min_x, double(v.x()));
            max_y = std::max(max_y, double(v.y()));
        }
    CHECK_THAT(min_x, WithinAbs(-0.5 * (f.side + f.boss), 1e-4));
    CHECK_THAT(max_y, WithinAbs(0.5 * (f.side + f.boss), 1e-4));
    // The upright frames stand up to their top bosses.
    CHECK_THAT(bounds(its).max.z(), WithinAbs(f.upright + f.boss, 1e-4));
}

TEST_CASE("The skew angle of a printed square is the machine's skew", "[CalibrationPrints]")
{
    const double  skew = GENERATE(-0.5, 0., 0.3, 2.);
    const Printed sq   = print_square(100., skew, 1.);
    CHECK_THAT(skew_angle((sq.c - sq.a).norm(), (sq.d - sq.b).norm(), (sq.d - sq.a).norm()), WithinAbs(skew, 1e-6));
}

TEST_CASE("Frames printed skewed and shrunk fit to that skew and scale", "[CalibrationPrints]")
{
    const SquareFrames f;
    const double       scale = 0.995, boss = f.boss * scale + 0.1; // bosses printed a bead 0.05 mm too wide all round
    auto reading = [&](double side, double skew) {
        const Printed sq = print_square(side, skew, scale);
        return FrameReading { (sq.c - sq.a).norm() + boss, (sq.d - sq.b).norm() + boss, (sq.d - sq.a).norm() + boss };
    };
    FramesFit   fit;
    std::string error;
    REQUIRE(fit_square_frames(f, boss, reading(f.side, 0.4), reading(f.upright, -0.2), reading(f.upright, 0.1), fit, error));
    // A leaning axis's side is longer by the square of the skew, 2.4e-5 at 0.4 degrees.
    CHECK_THAT(fit.scale_x, WithinAbs(scale, 1e-6));
    CHECK_THAT(fit.scale_y, WithinAbs(scale, 1e-4));
    CHECK_THAT(fit.scale_z, WithinAbs(scale, 1e-4));
    CHECK_THAT(fit.skew_xy, WithinAbs(0.4, 1e-6));
    CHECK_THAT(fit.skew_xz, WithinAbs(-0.2, 1e-6));
    CHECK_THAT(fit.skew_yz, WithinAbs(0.1, 1e-6));
}

TEST_CASE("Frame readings far from the frames printed are refused", "[CalibrationPrints]")
{
    const SquareFrames f;
    const FrameReading flat { 160., 160., 108. }, upright { 92.9, 92.9, 68. };
    FramesFit          fit;
    std::string        error;
    // A flat frame diagonal of 160 mm is 11 mm longer than the frame's, bosses and all.
    CHECK_FALSE(fit_square_frames(f, f.boss, flat, upright, upright, fit, error));
    CHECK_FALSE(error.empty());
}

TEST_CASE("X-Y compensation is half the mean shortfall of the features measured", "[CalibrationPrints]")
{
    double      out = 0.;
    std::string error;
    // Holes 0.2 and 0.3 mm small (0.1 and 0.15 each side); the 15 mm one not measured.
    REQUIRE(fit_xy_compensation({ 5., 10., 15., 20. }, { 4.8, 9.7, 0., 0. }, out, error));
    CHECK_THAT(out, WithinAbs(0.125, 1e-9));
    CHECK_FALSE(fit_xy_compensation({ 5., 10. }, { 0., 0. }, out, error));
    CHECK_FALSE(fit_xy_compensation({ 10. }, { 7. }, out, error));
}

TEST_CASE("Each overhang sample's ledge overhangs at its angle from vertical", "[CalibrationPrints]")
{
    const double angle = GENERATE(30., 55., 80.);
    const indexed_triangle_set its = overhang_samples({ angle }, 200.);
    bool found = false;
    for (const Vec3i32 &t : its.indices) {
        const Vec3d a = its.vertices[t[0]].cast<double>(), b = its.vertices[t[1]].cast<double>(), c = its.vertices[t[2]].cast<double>();
        const Vec3d n = (b - a).cross(c - a).normalized();
        // The underside faces down and out from the post.
        if (n.z() < -1e-3 && n.x() > 1e-3) {
            CHECK_THAT(std::atan2(-n.z(), n.x()) * 180. / M_PI, WithinAbs(angle, 1e-3));
            found = true;
        }
    }
    CHECK(found);
}

TEST_CASE("The holes and pegs plate is as wide as its features and their gaps", "[CalibrationPrints]")
{
    const std::vector<double> d = { 5., 10. };
    const BoundingBoxf3       box = bounds(holes_and_pegs(d));
    // 8 mm before, between and after the two features.
    CHECK_THAT(box.size().x(), WithinAbs(5. + 10. + 3. * 8., 1e-4));
    CHECK_THAT(box.center().x(), WithinAbs(0., 1e-4));
    CHECK_THAT(box.min.z(), WithinAbs(0., 1e-6));
}

TEST_CASE("A label is as wide as label_width says", "[CalibrationPrints]")
{
    // Characters drawn across their full width at both ends: 0, 2, 7, 5, A and C have a top bar.
    const std::string text = GENERATE("0.2", "75", "2.0", "AC");
    indexed_triangle_set its;
    append_label(its, text, 0., 0., 0., 5., 0.6);
    const BoundingBoxf3 box = bounds(its);
    CHECK_THAT(box.min.x(), WithinAbs(0., 1e-5));
    CHECK_THAT(box.size().x(), WithinAbs(label_width(text, 5.), 1e-5));
    CHECK_THAT(box.size().y(), WithinAbs(5., 1e-5));
    CHECK_THAT(box.size().z(), WithinAbs(0.6, 1e-5));
}

TEST_CASE("A label stays within label_width", "[CalibrationPrints]")
{
    // A 1 is drawn at the right of its place.
    const std::string text = GENERATE("1234567890", "Abd-");
    indexed_triangle_set its;
    append_label(its, text, 0., 0., 0., 5., 0.6);
    const BoundingBoxf3 box = bounds(its);
    CHECK(box.min.x() >= -1e-5);
    CHECK(box.max.x() <= label_width(text, 5.) + 1e-5);
}
