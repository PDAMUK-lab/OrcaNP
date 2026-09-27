#pragma once

// The non-planar and polar steps of the G-code export: G-code sliced on S4-deformed objects is
// mapped back into the real parts, and the result is converted to polar machine coordinates.
// See docs/HLSD/nonplanar-s4.md.

#include "../NonPlanar/PolarKinematics.hpp"
#include "../NonPlanar/S4GCodeTransform.hpp"
#include "../NonPlanar/S4Mapping.hpp"

#include <memory>
#include <vector>

namespace Slic3r {

class Print;
class PrintConfig;
class PrintObject;

namespace NonPlanarExport {

// True when any object of the print was deformed for S4.
bool has_s4(const Print &print);

// The object's id in its NONPLANAR_OBJECT markers: its place in the print's objects.
int marker_id(const Print &print, const PrintObject &object);

// How the G-code of each deformed object (in G-code coordinates) maps back into the real part,
// by the object ids of its NONPLANAR_OBJECT markers. Everything else is printed as sliced.
struct S4Mappers
{
    std::vector<std::unique_ptr<NonPlanar::S4Mapper>> storage;
    NonPlanar::S4MapperSet                            set;
};
std::unique_ptr<S4Mappers> s4_mappers(const Print &print);

NonPlanar::S4GCodeConfig         s4_gcode_config(const PrintConfig &config);
NonPlanar::PolarKinematicsConfig polar_config(const PrintConfig &config);

} // namespace NonPlanarExport
} // namespace Slic3r
