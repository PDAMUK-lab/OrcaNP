#pragma once

// The non-planar and polar steps of the G-code export: G-code sliced on S4-deformed objects is
// mapped back into the real parts, and the result is converted to polar machine coordinates.
// See docs/HLSD/nonplanar-s4.md.

#include "../NonPlanar/PolarKinematics.hpp"
#include "../NonPlanar/S4GCodeTransform.hpp"
#include "../NonPlanar/S4Mapping.hpp"

#include <memory>

namespace Slic3r {

class Print;
class PrintConfig;

namespace NonPlanarExport {

// True when any object of the print was deformed for S4.
bool has_s4(const Print &print);

// Maps G-code of all S4 objects of the print (in G-code coordinates) back into the real parts.
std::unique_ptr<NonPlanar::S4Mapper> s4_mapper(const Print &print);

NonPlanar::S4GCodeConfig         s4_gcode_config(const PrintConfig &config);
NonPlanar::PolarKinematicsConfig polar_config(const PrintConfig &config);

} // namespace NonPlanarExport
} // namespace Slic3r
