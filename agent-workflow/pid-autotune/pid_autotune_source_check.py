from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src" / "pid_autotune.cpp"
HDR = ROOT / "include" / "pid_autotune.hpp"
MAIN = ROOT / "src" / "main.cpp"


def require(text, needle, label):
    assert needle in text, f"missing {label}: {needle}"


def main():
    src = SRC.read_text(encoding="utf-8")
    hdr = HDR.read_text(encoding="utf-8")
    main_cpp = MAIN.read_text(encoding="utf-8")

    require(hdr, "void pid_autotune_auton();", "autotune declaration")
    require(src, "void pid_autotune_auton()", "autotune implementation")
    require(src, "pid_autotune_apply_if_ready", "autotune apply helper")
    require(src, "run_drive_trial", "drive trial")
    require(src, "run_turn_trial", "turn trial")
    require(src, "chassis.pid_drive_set", "EZ drive movement")
    require(src, "chassis.pid_turn_set", "EZ turn movement")
    require(src, "kDriveTrialTimeoutMs", "drive watchdog timeout")
    require(src, "kTurnTrialTimeoutMs", "turn watchdog timeout")
    require(src, "chassis.pid_targets_reset", "force cancel movement")
    require(src, "chassis.drive_mode_set(ez::DISABLE", "disable movement after trial")
    require(src, "kDriveSpeed", "drive speed")
    require(src, "kTurnSpeed", "turn speed")
    require(src, "PID_TUNE", "serial output")
    require(src, "chassis.pid_drive_constants_set", "drive constants apply")
    require(src, "chassis.pid_turn_constants_set", "turn constants apply")
    require(main_cpp, "#include \"pid_autotune.hpp\"", "main include")
    require(main_cpp, "start_pid_autotune", "opcontrol autotune launcher")
    require(main_cpp, "E_CONTROLLER_DIGITAL_X", "X hotkey")
    require(main_cpp, "E_CONTROLLER_DIGITAL_DOWN", "Down hotkey")

    print("pid autotune source checks passed")


if __name__ == "__main__":
    main()
