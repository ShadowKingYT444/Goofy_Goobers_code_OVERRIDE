#pragma once

#include "override_field.hpp"

#include "api.h"
#include "pros/ai_vision.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>

namespace avreset {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kImageWidthPx = 320.0;
constexpr double kImageHeightPx = 240.0;
constexpr double kDetectedTagSizeIn = 0.625;  // AI Vision inner Circle21h7 quad
constexpr double kGoalFaceOffsetIn = 5.61 / 2.0;
constexpr double kMinTagEdgePx = 5.0;
constexpr double kMaxTagEdgeRatio = 1.9;
constexpr double kMinTagFillRatio = 0.40;
constexpr double kMinQuadAreaPx2 = 35.0;
constexpr double kMinRangeIn = 1.5;
constexpr double kMaxRangeIn = 72.0;
constexpr double kMaxPositionInnovationIn = 18.0;
constexpr double kMaxHeadingInnovationDeg = 25.0;
constexpr double kHeadingScoreInPerDeg = 0.22;
constexpr double kMinCandidateMarginScore = 1.5;
constexpr double kMaxStablePositionStepIn = 2.0;
constexpr double kMaxStableHeadingStepDeg = 4.0;
constexpr double kMaxStablePositionSpreadIn = 1.5;
constexpr double kMaxStableHeadingSpreadDeg = 2.5;
constexpr std::uint32_t kTestPollMs = 50;
constexpr std::uint32_t kTestResetPeriodMs = 100;
constexpr int kRequiredStableSamples = 3;

// Override field geometry is kept in a separate header so the field map is easy
// to inspect or update without touching the estimator math.
using field::kGoals;
using field::kFaceNormals;
using field::kFaceNames;


struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct CameraTagPose {
    bool valid = false;
    Vec3 translation;      // camera -> tag center, camera frame, inches
    Vec3 inward_normal;    // tag inward normal in camera frame
    double planar_range = NAN;
    double bearing_deg = NAN;
    double normal_bearing_deg = NAN;
};

struct Measurement {
    bool valid = false;
    int tag_id = -1;
    int goal_index = -1;
    int face_index = -1;
    double x = NAN;
    double y = NAN;
    double theta = NAN;
    double range = NAN;
    double bearing_deg = NAN;
    double position_innovation = NAN;
    double heading_innovation = NAN;
    const char* reason = "invalid";
};

struct FilterState {
    int count = 0;
    int goal_index = -1;
    int face_index = -1;
    std::array<Measurement, kRequiredStableSamples> samples{};
};

struct SharedState {
    lemlib::Chassis* chassis = nullptr;
    std::uint8_t port = 0;
    CameraMount mount{};
    bool initialized = false;
    std::ofstream log;
    FilterState test_filter{};
    std::uint32_t last_test_poll_ms = 0;
    std::uint32_t last_test_reset_ms = 0;
};

// An external-linkage inline function has one function-local static object
// shared by every translation unit that includes this header. This makes the
// package truly header-only while keeping init() in main.cpp and reset() in
// autons.cpp on the same state.
inline SharedState& shared_state() {
    static SharedState state;
    return state;
}

#define g_chassis (shared_state().chassis)
#define g_port (shared_state().port)
#define g_mount (shared_state().mount)
#define g_initialized (shared_state().initialized)
#define g_log (shared_state().log)
#define g_test_filter (shared_state().test_filter)
#define g_last_test_poll_ms (shared_state().last_test_poll_ms)
#define g_last_test_reset_ms (shared_state().last_test_reset_ms)

inline double deg_to_rad(double deg) { return deg * kPi / 180.0; }
inline double rad_to_deg(double rad) { return rad * 180.0 / kPi; }

double normalize_deg(double deg) {
    while (deg >= 360.0) deg -= 360.0;
    while (deg < 0.0) deg += 360.0;
    return deg;
}

double angle_diff_deg(double target, double current) {
    double d = normalize_deg(target) - normalize_deg(current);
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

double vector_heading_deg(double x, double y) {
    return normalize_deg(rad_to_deg(std::atan2(x, y)));
}

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }

Vec3 scale(Vec3 a, double s) {
    a.x *= s;
    a.y *= s;
    a.z *= s;
    return a;
}

Vec3 subtract(Vec3 a, const Vec3& b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

Vec3 cross3(const Vec3& a, const Vec3& b) {
    return Vec3{a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
}

bool normalize(Vec3& v) {
    const double n = norm(v);
    if (!std::isfinite(n) || n < 1e-9) return false;
    v = scale(v, 1.0 / n);
    return true;
}

double cross2(int ax, int ay, int bx, int by, int cx, int cy) {
    return static_cast<double>(bx - ax) * static_cast<double>(cy - ay) -
           static_cast<double>(by - ay) * static_cast<double>(cx - ax);
}

double point_distance(int x0, int y0, int x1, int y1) {
    return std::hypot(static_cast<double>(x1 - x0),
                      static_cast<double>(y1 - y0));
}

double quad_area(const pros::aivision_object_tag_s_t& tag) {
    const double twice_area =
        static_cast<double>(tag.x0) * tag.y1 - static_cast<double>(tag.y0) * tag.x1 +
        static_cast<double>(tag.x1) * tag.y2 - static_cast<double>(tag.y1) * tag.x2 +
        static_cast<double>(tag.x2) * tag.y3 - static_cast<double>(tag.y2) * tag.x3 +
        static_cast<double>(tag.x3) * tag.y0 - static_cast<double>(tag.y3) * tag.x0;
    return std::abs(twice_area) * 0.5;
}

bool corners_in_bounds(const pros::aivision_object_tag_s_t& tag) {
    constexpr int margin = 1;
    const std::array<int, 8> values{{
        tag.x0, tag.y0, tag.x1, tag.y1,
        tag.x2, tag.y2, tag.x3, tag.y3,
    }};
    for (std::size_t i = 0; i < values.size(); i += 2) {
        if (values[i] <= margin ||
            values[i] >= static_cast<int>(kImageWidthPx) - 1 - margin ||
            values[i + 1] <= margin ||
            values[i + 1] >= static_cast<int>(kImageHeightPx) - 1 - margin) {
            return false;
        }
    }
    return true;
}

bool convex_quad(const pros::aivision_object_tag_s_t& tag) {
    const double c0 = cross2(tag.x0, tag.y0, tag.x1, tag.y1, tag.x2, tag.y2);
    const double c1 = cross2(tag.x1, tag.y1, tag.x2, tag.y2, tag.x3, tag.y3);
    const double c2 = cross2(tag.x2, tag.y2, tag.x3, tag.y3, tag.x0, tag.y0);
    const double c3 = cross2(tag.x3, tag.y3, tag.x0, tag.y0, tag.x1, tag.y1);
    return (c0 > 0 && c1 > 0 && c2 > 0 && c3 > 0) ||
           (c0 < 0 && c1 < 0 && c2 < 0 && c3 < 0);
}

bool geometry_usable(const pros::aivision_object_tag_s_t& tag) {
    if (!corners_in_bounds(tag) || !convex_quad(tag)) return false;
    const std::array<double, 4> edges{{
        point_distance(tag.x0, tag.y0, tag.x1, tag.y1),
        point_distance(tag.x1, tag.y1, tag.x2, tag.y2),
        point_distance(tag.x2, tag.y2, tag.x3, tag.y3),
        point_distance(tag.x3, tag.y3, tag.x0, tag.y0),
    }};
    const auto mm = std::minmax_element(edges.begin(), edges.end());
    if (*mm.first < kMinTagEdgePx || *mm.first <= 0.0 ||
        *mm.second / *mm.first > kMaxTagEdgeRatio) return false;

    const double area = quad_area(tag);
    if (area < kMinQuadAreaPx2) return false;
    const int min_x = std::min({tag.x0, tag.x1, tag.x2, tag.x3});
    const int max_x = std::max({tag.x0, tag.x1, tag.x2, tag.x3});
    const int min_y = std::min({tag.y0, tag.y1, tag.y2, tag.y3});
    const int max_y = std::max({tag.y0, tag.y1, tag.y2, tag.y3});
    const double box_area = static_cast<double>(max_x - min_x) *
                            static_cast<double>(max_y - min_y);
    return box_area > 0.0 && area / box_area >= kMinTagFillRatio;
}

bool choose_tag(pros::aivision_object_s_t& chosen) {
    const int count = pros::c::aivision_get_object_count(g_port);
    if (count <= 0 || count > 24) return false;
    bool found = false;
    double best_area = -1.0;
    for (int i = 0; i < count; ++i) {
        const auto object = pros::c::aivision_get_object(
            g_port, static_cast<std::uint32_t>(i));
        if (object.type != pros::E_AIVISION_DETECTED_TAG) continue;
        if (object.id > 4) continue;
        if (!geometry_usable(object.object.tag)) continue;
        const double area = quad_area(object.object.tag);
        if (area > best_area) {
            chosen = object;
            best_area = area;
            found = true;
        }
    }
    return found;
}

// Solve an 8x8 linear system in place with partial pivoting.
bool solve8(double a[8][9], double x[8]) {
    for (int col = 0; col < 8; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 8; ++row) {
            if (std::fabs(a[row][col]) > std::fabs(a[pivot][col])) pivot = row;
        }
        if (std::fabs(a[pivot][col]) < 1e-9) return false;
        if (pivot != col) {
            for (int j = col; j < 9; ++j) std::swap(a[pivot][j], a[col][j]);
        }
        const double div = a[col][col];
        for (int j = col; j < 9; ++j) a[col][j] /= div;
        for (int row = 0; row < 8; ++row) {
            if (row == col) continue;
            const double factor = a[row][col];
            for (int j = col; j < 9; ++j) a[row][j] -= factor * a[col][j];
        }
    }
    for (int i = 0; i < 8; ++i) x[i] = a[i][8];
    return true;
}

bool estimate_tag_pose(const pros::aivision_object_tag_s_t& tag,
                       CameraTagPose& out) {
    const double h = kDetectedTagSizeIn * 0.5;
    const std::array<std::array<double, 2>, 4> object{{
        {{-h, -h}}, {{h, -h}}, {{h, h}}, {{-h, h}},
    }};
    const std::array<std::array<double, 2>, 4> image{{
        {{static_cast<double>(tag.x0), static_cast<double>(tag.y0)}},
        {{static_cast<double>(tag.x1), static_cast<double>(tag.y1)}},
        {{static_cast<double>(tag.x2), static_cast<double>(tag.y2)}},
        {{static_cast<double>(tag.x3), static_cast<double>(tag.y3)}},
    }};

    double a[8][9]{};
    for (int i = 0; i < 4; ++i) {
        const double X = object[i][0];
        const double Y = object[i][1];
        const double u = image[i][0];
        const double v = image[i][1];
        const int r = 2 * i;
        a[r][0] = X; a[r][1] = Y; a[r][2] = 1.0;
        a[r][6] = -u * X; a[r][7] = -u * Y; a[r][8] = u;
        a[r + 1][3] = X; a[r + 1][4] = Y; a[r + 1][5] = 1.0;
        a[r + 1][6] = -v * X; a[r + 1][7] = -v * Y; a[r + 1][8] = v;
    }

    double q[8]{};
    if (!solve8(a, q)) return false;
    const double h11 = q[0], h12 = q[1], h13 = q[2];
    const double h21 = q[3], h22 = q[4], h23 = q[5];
    const double h31 = q[6], h32 = q[7];

    Vec3 b1{(h11 - g_mount.cx_px * h31) / g_mount.fx_px,
            (h21 - g_mount.cy_px * h31) / g_mount.fy_px,
            h31};
    Vec3 b2{(h12 - g_mount.cx_px * h32) / g_mount.fx_px,
            (h22 - g_mount.cy_px * h32) / g_mount.fy_px,
            h32};
    Vec3 b3{(h13 - g_mount.cx_px) / g_mount.fx_px,
            (h23 - g_mount.cy_px) / g_mount.fy_px,
            1.0};

    const double n1 = norm(b1);
    const double n2 = norm(b2);
    if (!std::isfinite(n1) || !std::isfinite(n2) || n1 < 1e-9 || n2 < 1e-9)
        return false;
    double lambda = 2.0 / (n1 + n2);
    Vec3 r1 = scale(b1, lambda);
    Vec3 r2 = scale(b2, lambda);
    Vec3 t = scale(b3, lambda);
    if (t.z < 0.0) {
        r1 = scale(r1, -1.0);
        r2 = scale(r2, -1.0);
        t = scale(t, -1.0);
    }

    if (!normalize(r1)) return false;
    r2 = subtract(r2, scale(r1, dot(r1, r2)));
    if (!normalize(r2)) return false;
    Vec3 r3 = cross3(r1, r2);
    if (!normalize(r3)) return false;
    // With the declared corner order, a fronto-parallel tag has r3 = +camera Z.
    if (r3.z < 0.0) r3 = scale(r3, -1.0);

    const double planar_range = std::hypot(t.x, t.z);
    const double bearing = rad_to_deg(std::atan2(t.x, t.z));
    const double normal_bearing = rad_to_deg(std::atan2(r3.x, r3.z));
    if (!std::isfinite(planar_range) || !std::isfinite(bearing) ||
        !std::isfinite(normal_bearing) || planar_range < kMinRangeIn ||
        planar_range > kMaxRangeIn) return false;

    out.valid = true;
    out.translation = t;
    out.inward_normal = r3;
    out.planar_range = planar_range;
    out.bearing_deg = bearing;
    out.normal_bearing_deg = normal_bearing;
    return true;
}

bool read_measurement(Measurement& out) {
    out = Measurement{};
    if (!g_initialized || g_chassis == nullptr) {
        out.reason = "not_initialized";
        return false;
    }

    pros::aivision_object_s_t object{};
    if (!choose_tag(object)) {
        out.reason = "no_good_tag";
        return false;
    }

    CameraTagPose camera_tag{};
    if (!estimate_tag_pose(object.object.tag, camera_tag)) {
        out.reason = "pose_math";
        return false;
    }

    const auto current = g_chassis->getPose();
    double best_score = std::numeric_limits<double>::infinity();
    double second_score = std::numeric_limits<double>::infinity();
    Measurement best{};

    for (std::size_t gi = 0; gi < kGoals.size(); ++gi) {
        if (kGoals[gi].tag_id != object.id) continue;
        for (std::size_t fi = 0; fi < kFaceNormals.size(); ++fi) {
            const double nx = kFaceNormals[fi][0];
            const double ny = kFaceNormals[fi][1];
            const double face_x = kGoals[gi].x + nx * kGoalFaceOffsetIn;
            const double face_y = kGoals[gi].y + ny * kGoalFaceOffsetIn;

            // r3 is the tag inward normal; globally that is -outward face normal.
            const double inward_heading = vector_heading_deg(-nx, -ny);
            const double camera_heading = normalize_deg(
                inward_heading - camera_tag.normal_bearing_deg);
            const double robot_heading = normalize_deg(camera_heading - g_mount.yaw_deg);

            const double ray_heading = normalize_deg(
                camera_heading + camera_tag.bearing_deg);
            const double ray_rad = deg_to_rad(ray_heading);
            const double camera_x = face_x - camera_tag.planar_range * std::sin(ray_rad);
            const double camera_y = face_y - camera_tag.planar_range * std::cos(ray_rad);

            const double robot_rad = deg_to_rad(robot_heading);
            const double mount_x = g_mount.forward_in * std::sin(robot_rad) +
                                   g_mount.right_in * std::cos(robot_rad);
            const double mount_y = g_mount.forward_in * std::cos(robot_rad) -
                                   g_mount.right_in * std::sin(robot_rad);
            const double robot_x = camera_x - mount_x;
            const double robot_y = camera_y - mount_y;

            const double pos_innovation = std::hypot(robot_x - current.x,
                                                      robot_y - current.y);
            const double heading_innovation = std::fabs(
                angle_diff_deg(robot_heading, current.theta));
            const double score = std::hypot(
                pos_innovation, heading_innovation * kHeadingScoreInPerDeg);

            if (score < best_score) {
                second_score = best_score;
                best_score = score;
                best.valid = true;
                best.tag_id = object.id;
                best.goal_index = static_cast<int>(gi);
                best.face_index = static_cast<int>(fi);
                best.x = robot_x;
                best.y = robot_y;
                best.theta = robot_heading;
                best.range = camera_tag.planar_range;
                best.bearing_deg = camera_tag.bearing_deg;
                best.position_innovation = pos_innovation;
                best.heading_innovation = heading_innovation;
                best.reason = "good";
            } else if (score < second_score) {
                second_score = score;
            }
        }
    }

    if (!best.valid) {
        out.reason = "no_face_candidate";
        return false;
    }
    if (best.position_innovation > kMaxPositionInnovationIn) {
        out = best;
        out.valid = false;
        out.reason = "position_gate";
        return false;
    }
    if (best.heading_innovation > kMaxHeadingInnovationDeg) {
        out = best;
        out.valid = false;
        out.reason = "heading_gate";
        return false;
    }
    if (std::isfinite(second_score) &&
        second_score - best_score < kMinCandidateMarginScore) {
        out = best;
        out.valid = false;
        out.reason = "ambiguous_goal_face";
        return false;
    }

    out = best;
    return true;
}

void clear_filter(FilterState& filter) { filter = FilterState{}; }

double circular_mean_deg(const std::array<Measurement, kRequiredStableSamples>& s) {
    double ss = 0.0, cc = 0.0;
    for (const auto& m : s) {
        const double r = deg_to_rad(m.theta);
        ss += std::sin(r);
        cc += std::cos(r);
    }
    return normalize_deg(rad_to_deg(std::atan2(ss, cc)));
}

bool add_to_filter(FilterState& filter, const Measurement& m,
                   double& x, double& y, double& theta) {
    if (!m.valid) {
        clear_filter(filter);
        return false;
    }
    if (filter.count != 0 &&
        (filter.goal_index != m.goal_index || filter.face_index != m.face_index)) {
        clear_filter(filter);
    }
    if (filter.count > 0) {
        const auto& last = filter.samples[filter.count - 1];
        if (std::hypot(m.x - last.x, m.y - last.y) > kMaxStablePositionStepIn ||
            std::fabs(angle_diff_deg(m.theta, last.theta)) > kMaxStableHeadingStepDeg) {
            clear_filter(filter);
        }
    }
    if (filter.count == 0) {
        filter.goal_index = m.goal_index;
        filter.face_index = m.face_index;
    }
    if (filter.count < kRequiredStableSamples) {
        filter.samples[filter.count++] = m;
    } else {
        for (int i = 1; i < kRequiredStableSamples; ++i)
            filter.samples[i - 1] = filter.samples[i];
        filter.samples[kRequiredStableSamples - 1] = m;
    }
    if (filter.count < kRequiredStableSamples) return false;

    x = 0.0;
    y = 0.0;
    for (const auto& sample : filter.samples) {
        x += sample.x;
        y += sample.y;
    }
    x /= kRequiredStableSamples;
    y /= kRequiredStableSamples;
    theta = circular_mean_deg(filter.samples);

    double max_pos_spread = 0.0;
    double max_heading_spread = 0.0;
    for (const auto& sample : filter.samples) {
        max_pos_spread = std::max(max_pos_spread,
            std::hypot(sample.x - x, sample.y - y));
        max_heading_spread = std::max(max_heading_spread,
            std::fabs(angle_diff_deg(sample.theta, theta)));
    }
    return max_pos_spread <= kMaxStablePositionSpreadIn &&
           max_heading_spread <= kMaxStableHeadingSpreadDeg;
}

void open_log() {
    if (g_log.is_open()) return;
    g_log.open("/usd/aivision_reset.csv", std::ios::out | std::ios::trunc);
    if (g_log.is_open()) {
        g_log << "ms,mode,tag_id,goal,face,odom_x,odom_y,odom_theta,vision_x,vision_y,vision_theta,pos_innovation,heading_innovation,range,accepted,reason\n";
        g_log.flush();
    }
}

void log_row(const char* mode, const lemlib::Pose& odom,
             const Measurement& m, bool accepted,
             double applied_x = NAN, double applied_y = NAN,
             double applied_theta = NAN) {
    if (!g_log.is_open()) return;
    const double vx = std::isfinite(applied_x) ? applied_x : m.x;
    const double vy = std::isfinite(applied_y) ? applied_y : m.y;
    const double vt = std::isfinite(applied_theta) ? applied_theta : m.theta;
    const char* goal = (m.goal_index >= 0 &&
                        m.goal_index < static_cast<int>(kGoals.size()))
                           ? kGoals[m.goal_index].name : "none";
    const char* face = (m.face_index >= 0 && m.face_index < 4)
                           ? kFaceNames[m.face_index] : "none";
    g_log << pros::millis() << ',' << mode << ',' << m.tag_id << ',' << goal << ','
          << face << ',' << odom.x << ',' << odom.y << ',' << odom.theta << ','
          << vx << ',' << vy << ',' << vt << ',' << m.position_innovation << ','
          << m.heading_innovation << ',' << m.range << ','
          << (accepted ? 1 : 0) << ',' << m.reason << '\n';
    g_log.flush();
}

void apply_reset(double x, double y, double theta,
                 const Measurement& m, const char* mode) {
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(theta)){
        std::printf("AVRESET REJECT: non-finite pose\n");
        std::fflush(stdout);
        return;
    }
    const auto before = g_chassis->getPose();
    g_chassis->setPose(x, y, theta);
    log_row(mode, before, m, true, x, y, theta);
    std::printf(
        "AVRESET accepted mode=%s tag=%d goal=%s face=%s old=(%.2f,%.2f,%.1f) new=(%.2f,%.2f,%.1f) range=%.2f dpos=%.2f dtheta=%.2f\n",
        mode, m.tag_id,
        (m.goal_index >= 0 ? kGoals[m.goal_index].name : "none"),
        (m.face_index >= 0 ? kFaceNames[m.face_index] : "none"),
        before.x, before.y, before.theta, x, y, theta, m.range,
        m.position_innovation, m.heading_innovation);
    std::fflush(stdout);
}

}  // namespace

inline void init(lemlib::Chassis& chassis, std::uint8_t ai_vision_port,
          CameraMount mount) {
    g_chassis = &chassis;
    g_port = ai_vision_port;
    g_mount = mount;
    g_initialized = true;
    clear_filter(g_test_filter);
    g_last_test_poll_ms = 0;
    g_last_test_reset_ms = 0;
    pros::c::aivision_enable_detection_types(g_port, pros::E_AIVISION_MODE_TAGS);
    open_log();
    std::printf(
        "AVRESET init port=%u forward=%.2f right=%.2f yaw=%.1f fx=%.2f fy=%.2f log=%s\n",
        static_cast<unsigned>(g_port), g_mount.forward_in, g_mount.right_in,
        g_mount.yaw_deg, g_mount.fx_px, g_mount.fy_px,
        g_log.is_open() ? "/usd/aivision_reset.csv" : "serial_only");
    std::fflush(stdout);
}

inline bool reset() {
    if (!g_initialized || g_chassis == nullptr) return false;
    FilterState filter{};
    Measurement last{};
    for (int attempt = 0; attempt < 14; ++attempt) {
        Measurement m{};
        const bool good = read_measurement(m);
        last = m;
        if (good) {
            double x = NAN, y = NAN, theta = NAN;
            if (add_to_filter(filter, m, x, y, theta)) {
                apply_reset(x, y, theta, m, "auto");
                return true;
            }
        } else {
            clear_filter(filter);
        }
        pros::delay(30);
    }
    log_row("auto", g_chassis->getPose(), last, false);
    std::printf("AVRESET rejected mode=auto reason=%s\n", last.reason);
    std::fflush(stdout);
    return false;
}

inline void test() {
    if (!g_initialized || g_chassis == nullptr) return;
    const std::uint32_t now = pros::millis();
    if (now - g_last_test_poll_ms < kTestPollMs) return;
    g_last_test_poll_ms = now;

    const auto before = g_chassis->getPose();
    Measurement m{};
    if (!read_measurement(m)) {
        clear_filter(g_test_filter);
        log_row("test", before, m, false);
        return;
    }

    double x = NAN, y = NAN, theta = NAN;
    const bool stable = add_to_filter(g_test_filter, m, x, y, theta);
    if (stable && now - g_last_test_reset_ms >= kTestResetPeriodMs) {
        g_last_test_reset_ms = now;
        apply_reset(x, y, theta, m, "test");
    } else {
        log_row("test", before, m, false);
    }
}

#undef g_chassis
#undef g_port
#undef g_mount
#undef g_initialized
#undef g_log
#undef g_test_filter
#undef g_last_test_poll_ms
#undef g_last_test_reset_ms

}  // namespace avreset
