#pragma once

// Minimal G-code line parsing shared by the non-planar G-code passes.

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace Slic3r {
namespace NonPlanar {
namespace GCodeWords {

struct Line
{
    std::string code;    // without comment, trimmed
    std::string comment; // including the leading ';', or empty
};

inline Line split(const std::string &line)
{
    Line         l;
    const size_t semi = line.find(';');
    l.code = semi == std::string::npos ? line : line.substr(0, semi);
    if (semi != std::string::npos)
        l.comment = line.substr(semi);
    while (! l.code.empty() && std::isspace((unsigned char) l.code.back()))
        l.code.pop_back();
    size_t first = 0;
    while (first < l.code.size() && std::isspace((unsigned char) l.code[first]))
        ++first;
    l.code.erase(0, first);
    return l;
}

// The command ("G1", "M83", ...) with leading zeros removed from its number; empty for dotted
// commands such as G92.1 and for lines without a command.
inline std::string command(const std::string &code)
{
    if (code.empty() || ! std::isalpha((unsigned char) code[0]))
        return {};
    size_t i = 1;
    while (i < code.size() && std::isdigit((unsigned char) code[i]))
        ++i;
    if (i == 1 || (i < code.size() && code[i] == '.'))
        return {};
    std::string num = code.substr(1, i - 1);
    num.erase(0, std::min(num.find_first_not_of('0'), num.size() - 1));
    return std::string(1, (char) std::toupper((unsigned char) code[0])) + num;
}

// Value of the address word `letter` after the command, if present.
inline bool find(const std::string &code, char letter, double &value)
{
    size_t i = 1;
    while (i < code.size() && (std::isdigit((unsigned char) code[i]) || code[i] == '.'))
        ++i;
    for (; i < code.size(); ++i) {
        if (std::toupper((unsigned char) code[i]) != letter)
            continue;
        const char  *start = code.c_str() + i + 1;
        char        *end   = nullptr;
        const double v     = std::strtod(start, &end);
        if (end != start) {
            value = v;
            return true;
        }
    }
    return false;
}

// Fixed-point with trailing zeros removed.
inline std::string number(double v, int decimals)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    std::string s(buf);
    if (s.find('.') != std::string::npos) {
        while (s.back() == '0')
            s.pop_back();
        if (s.back() == '.')
            s.pop_back();
    }
    if (s == "-0")
        s = "0";
    return s;
}

} // namespace GCodeWords
} // namespace NonPlanar
} // namespace Slic3r
