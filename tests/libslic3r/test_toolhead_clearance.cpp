#include <catch2/catch_all.hpp>

#include "libslic3r/NonPlanar/ToolheadClearance.hpp"
#include "libslic3r/BoundingBox.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::NonPlanar;
using Catch::Matchers::WithinAbs;

namespace {

double deg(double rad) { return rad * 180. / PI; }

BoundingBoxf3 bounds(const indexed_triangle_set &its)
{
    BoundingBoxf3 box;
    for (const Vec3f &v : its.vertices)
        box.merge(v.cast<double>());
    return box;
}

ToolheadClearance toolhead(double tip_diameter, double angle, double length, double radius)
{
    ToolheadClearance t;
    t.tip_diameter = tip_diameter;
    t.angle        = angle;
    t.length       = length;
    t.radius       = radius;
    return t;
}

} // namespace

TEST_CASE("The modelled toolhead's underside rises from the tip's edge at the clearance angle up to the nozzle length",
          "[ToolheadClearance]")
{
    // A 0.8 mm tip, 45 degrees, 5 mm long, 20 mm in radius.
    const ToolheadClearance t = toolhead(0.8, 45., 5., 20.);
    CHECK_THAT(t.underside(0.3), WithinAbs(0., 1e-9));
    CHECK_THAT(t.underside(1.4), WithinAbs(1.0, 1e-9)); // 1 mm past the tip's edge at 45 degrees
    CHECK_THAT(t.underside(10.), WithinAbs(5., 1e-9));  // above the cone: the cylinder's base
    CHECK_THAT(t.reach(), WithinAbs(20., 1e-9));
    CHECK(std::isinf(t.underside(25.)));                // beyond the toolhead
}

TEST_CASE("A cone wider than the toolhead radius sets the reach", "[ToolheadClearance]")
{
    // 10 degrees for 5 mm reaches 0.4 + 5 / tan(10) = 28.8 mm, past the 20 mm radius.
    const ToolheadClearance t = toolhead(0.8, 10., 5., 20.);
    CHECK_THAT(t.reach(), WithinAbs(0.4 + 5. / std::tan(10. * PI / 180.), 1e-9));
}

TEST_CASE("Feeler readings keep the modelled toolhead under every reading", "[ToolheadClearance]")
{
    // A flat-bottomed toolhead: gaps of about a millimetre some 30 mm out, lowest at the front.
    const std::vector<FeelerReading> readings = { { 1.2, 32.75 }, { 2.0, 30. }, { 1.4, 32.75 }, { 1.6, 31. } };
    ToolheadClearance t;
    std::string       error;
    REQUIRE(fit_feeler_readings(readings, 0.8, 0., t, error));
    // The clearance angle is the shallowest rise from the tip's edge to a reading, rounded down.
    const double shallowest = deg(std::atan2(1.2, 32.75 - 0.4));
    CHECK(t.angle <= shallowest);
    CHECK(t.angle > shallowest - 0.1);
    // The toolhead reaches out to the farthest reading, and its underside stays under each reading.
    CHECK(t.reach() >= 32.75);
    for (const FeelerReading &r : readings)
        DYNAMIC_SECTION("gap " << r.gap << " at " << r.distance) { CHECK(t.underside(r.distance) <= r.gap); }
}

TEST_CASE("A toolhead reach beyond the feeler readings sets the toolhead radius", "[ToolheadClearance]")
{
    ToolheadClearance t;
    std::string       error;
    REQUIRE(fit_feeler_readings({ { 1., 20. } }, 0.8, 40., t, error));
    CHECK(t.radius >= 40.);
    // The cone reaches the radius, so the underside keeps rising at the angle all the way out.
    CHECK_THAT(t.underside(39.), WithinAbs((39. - 0.4) * std::tan(t.angle * PI / 180.), 0.01));
}

TEST_CASE("A feeler reading at the nozzle tip is refused", "[ToolheadClearance]")
{
    ToolheadClearance t;
    std::string       error;
    // 0.9 mm from the centre of a 0.8 mm tip is within 1 mm of its edge.
    CHECK_FALSE(fit_feeler_readings({ { 1., 0.9 } }, 0.8, 0., t, error));
    CHECK_FALSE(error.empty());
    CHECK_FALSE(fit_feeler_readings({ { 0., 20. } }, 0.8, 0., t, error));
    CHECK_FALSE(fit_feeler_readings({}, 0.8, 0., t, error));
}

TEST_CASE("Wedge readings take the tightest side's angle out to the toolhead reach", "[ToolheadClearance]")
{
    ToolheadClearance t;
    std::string       error;
    REQUIRE(fit_wedge_readings({ 60., 55., 40., 60. }, 0.8, 30., t, error));
    CHECK_THAT(t.angle, WithinAbs(40., 1e-9));
    CHECK_THAT(t.radius, WithinAbs(30., 1e-9));
    // The cone reaches the reach: 29.6 mm past the tip's edge at 40 degrees, rounded down.
    CHECK(t.length <= 29.6 * std::tan(40. * PI / 180.));
    CHECK(t.length > 29.6 * std::tan(40. * PI / 180.) - 0.01);
}

TEST_CASE("Wedge readings without a toolhead reach or with an impossible angle are refused", "[ToolheadClearance]")
{
    ToolheadClearance t;
    std::string       error;
    CHECK_FALSE(fit_wedge_readings({ 40. }, 0.8, 0., t, error));
    CHECK_FALSE(fit_wedge_readings({ 90. }, 0.8, 30., t, error));
    CHECK_FALSE(fit_wedge_readings({ 0. }, 0.8, 30., t, error));
    CHECK_FALSE(fit_wedge_readings({}, 0.8, 30., t, error));
}

TEST_CASE("The gauges and the fin test are closed surfaces", "[ToolheadClearance]")
{
    CHECK(its_num_open_edges(feeler_blades(feeler_thicknesses(), 150.)) == 0);
    CHECK(its_num_open_edges(angle_wedges(wedge_angles(), 30., 150.)) == 0);
    CHECK(its_num_open_edges(fin_test(toolhead(0.8, 45., 20., 20.), 1.)) == 0);
}

TEST_CASE("The gauges are laid out within the width given and centred", "[ToolheadClearance]")
{
    const double width = GENERATE(120., 200.);
    for (const indexed_triangle_set &its : { feeler_blades(feeler_thicknesses(), width), angle_wedges(wedge_angles(), 30., width) }) {
        const BoundingBoxf3 box = bounds(its);
        CHECK(box.size().x() <= width);
        CHECK_THAT(box.center().x(), WithinAbs(0., 1e-3));
        CHECK_THAT(box.center().y(), WithinAbs(0., 1e-3));
        CHECK_THAT(box.min.z(), WithinAbs(0., 1e-6));
    }
}

TEST_CASE("Every feeler blade is as thick as its number", "[ToolheadClearance]")
{
    // A single blade: its thin end, the far end from the handle, is the thickness.
    const double t = GENERATE(0.2, 1.4, 6.0);
    const indexed_triangle_set its = feeler_blades({ t }, 200.);
    const BoundingBoxf3        box = bounds(its);
    double                     tip_top = 0.;
    for (const Vec3f &v : its.vertices)
        if (v.x() < box.min.x() + 1e-3)
            tip_top = std::max(tip_top, double(v.z()));
    CHECK_THAT(tip_top, WithinAbs(t, 1e-5));
}

TEST_CASE("Each wedge's slope rises at its angle", "[ToolheadClearance]")
{
    const double angle = GENERATE(5., 40., 75.);
    const indexed_triangle_set its = angle_wedges({ angle }, 30., 200.);
    // The slope is the one face leaning towards -X.
    bool found = false;
    for (const Vec3i32 &f : its.indices) {
        const Vec3d a = its.vertices[f[0]].cast<double>(), b = its.vertices[f[1]].cast<double>(), c = its.vertices[f[2]].cast<double>();
        const Vec3d n = (b - a).cross(c - a).normalized();
        if (n.x() < -1e-3 && n.z() > 1e-3) {
            CHECK_THAT(deg(std::atan2(-n.x(), n.z())), WithinAbs(angle, 1e-3));
            found = true;
        }
    }
    CHECK(found);
}

TEST_CASE("The fin test's fins stay below the modelled toolhead by the margin", "[ToolheadClearance]")
{
    // A steep tilting toolhead, and a flat-bottomed one (2.2 degrees out to 32.75 mm).
    const ToolheadClearance t = GENERATE(toolhead(0.8, 45., 20., 20.), toolhead(0.8, 2.2, 1.24, 32.75), toolhead(1.2, 30., 5., 25.));
    const indexed_triangle_set its = fin_test(t, 1.);
    size_t fin_points = 0, outer_points = 0;
    for (const Vec3f &v : its.vertices) {
        const double d = v.head<2>().cast<double>().norm(), z = v.z() - fin_test_plate;
        if (z <= 1e-6)
            continue;
        if (d <= t.reach()) {
            ++ fin_points;
            CHECK(z <= t.underside(d) - fin_test_margin + 1e-5);
        } else if (d > t.reach() + 1.5) {
            // Beyond the reach the fins stand above the nozzle length, for a toolhead wider than its radius.
            ++ outer_points;
            CHECK(z >= std::min(t.length + 3., 30.) - 1e-5);
        }
    }
    CHECK(fin_points > 0);
    CHECK(outer_points > 0);
}

TEST_CASE("A label is as wide as label_width says", "[ToolheadClearance]")
{
    // Digits drawn across their full width at both ends: 0, 2, 7 and 5 have a top bar.
    const std::string text = GENERATE("0.2", "75", "2.0");
    indexed_triangle_set its;
    append_label(its, text, 0., 0., 0., 5., 0.6);
    const BoundingBoxf3 box = bounds(its);
    CHECK_THAT(box.min.x(), WithinAbs(0., 1e-5));
    CHECK_THAT(box.size().x(), WithinAbs(label_width(text, 5.), 1e-5));
    CHECK_THAT(box.size().y(), WithinAbs(5., 1e-5));
    CHECK_THAT(box.size().z(), WithinAbs(0.6, 1e-5));
}

TEST_CASE("A label stays within label_width", "[ToolheadClearance]")
{
    // A 1 is drawn at the right of its place.
    indexed_triangle_set its;
    append_label(its, "1234567890", 0., 0., 0., 5., 0.6);
    const BoundingBoxf3 box = bounds(its);
    CHECK(box.min.x() >= -1e-5);
    CHECK(box.max.x() <= label_width("1234567890", 5.) + 1e-5);
}
