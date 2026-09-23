#pragma once

#include <array>

namespace avreset::field {

// Coordinate convention used by this package and the supplied autons:
// field center = (0,0), +X = right, +Y = top, LemLib heading 0 = +Y.
struct Goal {
    int tag_id;
    double x;
    double y;
    const char* name;
};

// Override Goal centers. IDs 1-4 each occur on two mirrored Goals.
inline constexpr std::array<Goal, 9> kGoals{{
    {0,   0.00,   0.00, "center"},
    {4, -23.55,  47.10, "top_left"},
    {3,  23.54,  47.10, "top_right"},
    {1, -47.10,  23.55, "left_upper"},
    {2,  47.09,  23.55, "right_upper"},
    {2, -47.10, -23.54, "left_lower"},
    {1,  47.09, -23.54, "right_lower"},
    {3, -23.55, -47.09, "bottom_left"},
    {4,  23.54, -47.09, "bottom_right"},
}};

// Outward normals of the four tagged cardinal Goal faces.
inline constexpr std::array<std::array<double, 2>, 4> kFaceNormals{{
    {{1.0, 0.0}}, {{0.0, 1.0}}, {{-1.0, 0.0}}, {{0.0, -1.0}},
}};
inline constexpr std::array<const char*, 4> kFaceNames{{"+x", "+y", "-x", "-y"}};

}  // namespace avreset::field
