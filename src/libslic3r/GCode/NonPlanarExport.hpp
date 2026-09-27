#pragma once

// The non-planar and polar steps of the G-code export: G-code sliced on S4-deformed objects is
// mapped back into the real parts, and the result is converted to polar machine coordinates.
// See docs/HLSD/nonplanar-s4.md.

#include "../NonPlanar/PolarKinematics.hpp"
#include "../NonPlanar/S4GCodeTransform.hpp"
#include "../NonPlanar/S4Mapping.hpp"

#include <memory>
#include <string>
#include <vector>

namespace Slic3r {

class Print;
class PrintConfig;
class PrintObject;
struct GCodeProcessorResult;

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

// The preview's toolhead: the nozzle and head the clearance check keeps off the printed part.
void set_toolhead_preview(GCodeProcessorResult &result, const PrintConfig &config);
// For polar machine G-code: the machine pose at the end of each processed move, with the moves'
// line ids and the line ends moved from the Cartesian G-code the converter read to the machine
// G-code it wrote to `polar_path`, so the G-code window shows what the machine runs.
void map_preview_to_polar(GCodeProcessorResult &result, const NonPlanar::PolarGCodeConverter &converter, const PrintConfig &config,
                          const std::string &polar_path);

} // namespace NonPlanarExport
} // namespace Slic3r
