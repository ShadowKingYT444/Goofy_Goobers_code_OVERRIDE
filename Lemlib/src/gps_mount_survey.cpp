#include "autons.hpp"
#include "main.h"
#include "field_display_status.hpp"
#include "gps_reset/gps_reset.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {
constexpr int kSurveyStops = 8;
constexpr double kTargetStepDeg = 360.0 / kSurveyStops;
constexpr double kTargetToleranceDeg = 18.0;
constexpr int kAverageSamples = 24;
constexpr std::uint32_t kAveragePeriodMs = 50;
constexpr double kAcceptRmsIn = 1.0;
constexpr double kAcceptMaxResidualIn = 2.0;

void stop_and_coast() {
    chassis.cancelAllMotions();
    left_motors.move(0);
    right_motors.move(0);
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

bool newly_pressed(pros::Controller& controller,
                   pros::controller_digital_e_t button,
                   bool& previous) {
    const bool current = controller.get_digital(button);
    const bool pressed = current && !previous;
    previous = current;
    return pressed;
}

double max_center_residual(const std::vector<gpsreset::GpsPose>& samples,
                           const gpsreset::MountFit& fit) {
    double largest = 0.0;
    for (const auto& sample : samples) {
        const auto center = gpsreset::lens_to_center(
            sample, fit.forward_in, fit.right_in);
        largest = std::max(
            largest,
            std::hypot(center.x_in - fit.center_x_in,
                       center.y_in - fit.center_y_in));
    }
    return largest;
}
}  // namespace

// Survey the GPS lens offset without assuming an autonomous turn keeps the
// robot's rotation center fixed. Run from driver control with LEFT after
// building AUTON_ROUTINE=gps_mount_survey_auton.
//
// Tape a cross under the drivetrain rotation center. For each stop, manually
// rotate and re-center that exact point on the cross, then press A. The routine
// averages one second of GPS data and prints CSV rows for the offline fitter.
void gps_mount_survey_auton() {
    stop_and_coast();
    gpsreset::set_auto_reset(false);
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
        pros::delay(20);

    auto& state = gpsreset::shared_state();
    if (!state.initialized || state.gps == nullptr) {
        fieldviz::report("GPS OFFSET SURVEY\n\nFAILED: GPS not initialized");
        std::printf("GPS_MOUNT_SURVEY,NO_GPS\n");
        return;
    }

    std::vector<gpsreset::GpsPose> samples;
    samples.reserve(kSurveyStops);
    double first_heading = 0.0;
    bool previous_a = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool previous_b = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    for (int index = 0; index < kSurveyStops;) {
        const auto live = gpsreset::read_lens_pose(*state.gps);
        const double target = gpsreset::normalize_deg(
            first_heading + index * kTargetStepDeg);
        const double target_error = live.ok
            ? std::abs(gpsreset::wrap_deg(live.theta_deg - target))
            : 999.0;
        fieldviz::report(
            "GPS OFFSET SURVEY\n"
            "Stop %d of %d\n"
            "Center robot on tape cross\n"
            "Target heading %.0f deg\n"
            "Live heading %s%.0f deg\n"
            "A = sample   B = abort",
            index + 1, kSurveyStops, target,
            live.ok ? "" : "NO FIX / ", live.ok ? live.theta_deg : 0.0);

        if (newly_pressed(controller, pros::E_CONTROLLER_DIGITAL_B,
                          previous_b)) {
            fieldviz::report("GPS OFFSET SURVEY\n\nABORTED");
            std::printf("GPS_MOUNT_SURVEY,ABORTED,%d\n", index);
            return;
        }
        if (!newly_pressed(controller, pros::E_CONTROLLER_DIGITAL_A,
                           previous_a)) {
            pros::delay(20);
            continue;
        }
        if (!live.ok) {
            fieldviz::report("GPS OFFSET SURVEY\n\nNo confident fix.\nReposition and retry.");
            pros::delay(600);
            continue;
        }
        if (index > 0 && target_error > kTargetToleranceDeg) {
            fieldviz::report(
                "GPS OFFSET SURVEY\n\nHeading %.0f is too far\nfrom target %.0f. Retry.",
                live.theta_deg, target);
            pros::delay(700);
            continue;
        }

        gpsreset::GpsPose averaged{};
        fieldviz::report("GPS OFFSET SURVEY\n\nHold still; averaging...\nStop %d of %d",
                         index + 1, kSurveyStops);
        if (!gpsreset::average_lens_pose(
                averaged, kAverageSamples, kAveragePeriodMs)) {
            fieldviz::report("GPS OFFSET SURVEY\n\nFix lost during average.\nRetry this stop.");
            std::printf("GPS_MOUNT_SAMPLE_REJECTED,%d,NO_FIX\n", index + 1);
            pros::delay(700);
            continue;
        }

        if (index == 0) first_heading = averaged.theta_deg;
        samples.push_back(averaged);
        std::printf("GPS_MOUNT_SAMPLE,%d,%.5f,%.5f,%.5f\n",
                    index + 1, averaged.x_in, averaged.y_in,
                    averaged.theta_deg);
        std::fflush(stdout);
        ++index;
        pros::delay(300);
    }

    const auto fit = gpsreset::fit_mount(samples);
    if (!fit.ok) {
        fieldviz::report(
            "GPS OFFSET SURVEY\n\nFAILED: heading coverage\nor fit was singular.");
        std::printf("GPS_MOUNT_FIT,FAILED\n");
        return;
    }

    const double max_residual = max_center_residual(samples, fit);
    const bool accepted = fit.rms_in <= kAcceptRmsIn &&
                          max_residual <= kAcceptMaxResidualIn;
    std::printf(
        "GPS_MOUNT_FIT,%s,forward=%.5f,right=%.5f,center_x=%.5f,"
        "center_y=%.5f,rms=%.5f,max=%.5f,current_forward=%.5f,"
        "current_right=%.5f,facing=%.2f\n",
        accepted ? "PASS" : "REPEAT", fit.forward_in, fit.right_in,
        fit.center_x_in, fit.center_y_in, fit.rms_in, max_residual,
        state.mount_forward_in, state.mount_right_in,
        state.mount_facing_deg);
    std::fflush(stdout);

    fieldviz::report(
        "GPS OFFSET SURVEY\n"
        "%s\n"
        "Current fwd %.2f right %.2f\n"
        "Fit     fwd %.2f right %.2f\n"
        "RMS %.2f in  max %.2f in\n"
        "Copy GPS_MOUNT_SAMPLE rows\n"
        "to tools/fit_gps_mount.py",
        accepted ? "PASS" : "REPEAT TEST",
        state.mount_forward_in, state.mount_right_in,
        fit.forward_in, fit.right_in, fit.rms_in, max_residual);
}
