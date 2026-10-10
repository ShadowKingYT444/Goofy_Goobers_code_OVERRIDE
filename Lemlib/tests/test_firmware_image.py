"""Validate the linked hot/cold firmware after `pros make clean && pros make`.

This catches missing competition entry points and a missing display backend;
it cannot replace a boot test on the V5 Brain.
"""

from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]


def symbols(name):
    return subprocess.check_output(
        ["arm-none-eabi-nm", "-C", str(ROOT / "bin" / name)], text=True)


class FirmwareImageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # The hot link imports every cold symbol as absolute; drop those so
        # the table describes only what the hot image itself contains.
        cls.hot = "\n".join(line for line in symbols("hot.package.elf").splitlines()
                            if " A " not in line) + "\n"
        cls.cold = symbols("cold.package.elf")

    def defined(self, table, name):
        return [line for line in table.splitlines()
                if line.endswith(" T " + name)]

    def test_hot_cold_upload(self):
        self.assertGreater((ROOT / "bin/hot.package.bin").stat().st_size, 0)
        self.assertGreater((ROOT / "bin/cold.package.bin").stat().st_size, 0)
        # The single-image build of this project did not start on the Brain,
        # and a stale one must not be left where it could be uploaded.
        self.assertFalse((ROOT / "bin/monolith.bin").exists())

    def test_competition_entry_points(self):
        for name in ("initialize", "autonomous", "opcontrol", "disabled",
                     "competition_initialize", "install_hot_table"):
            self.assertEqual(len(self.defined(self.hot, name)), 1, name)

    def test_real_display_backend_is_in_cold_package(self):
        for name in ("display_initialize", "lv_init", "lv_timer_handler",
                     "lv_timer_create", "lv_label_set_text",
                     "lv_line_set_points"):
            self.assertEqual(len(self.defined(self.cold, name)), 1, name)

    def test_renderer_runs_from_an_lvgl_timer(self):
        # No linker wrapping: the daemon in the cold package calls the stock
        # handler, which runs the hot image's render timer.
        self.assertNotIn("__wrap_lv_timer_handler", self.hot + self.cold)
        self.assertIn("render(_lv_timer_t*)", self.hot)
        makefile = (ROOT / "Makefile").read_text()
        self.assertIn("USE_PACKAGE:=1", makefile)
        self.assertNotIn("--wrap", makefile)
        self.assertNotIn("--whole-archive", makefile)
        # LLEMU would introduce cross-task LVGL mutations and another screen.
        for source in (ROOT / "src").glob("*.cpp"):
            self.assertNotIn("pros::lcd::", source.read_text(), source.name)

    def test_unused_selector_is_not_linked(self):
        for table in (self.hot, self.cold):
            self.assertNotIn("ts::selector", table)
            self.assertNotIn("_GLOBAL__sub_I_registry_internal", table)

    def test_no_external_tracking_wheels(self):
        for name in ("vertical_odom", "horizontal_odom", "vertical_wheel",
                     "horizontal_wheel"):
            self.assertNotIn(" " + name + "\n", self.hot)
        for name in ("lift_sensor", "claw_sensor", "imu"):
            self.assertIn(" " + name + "\n", self.hot)


if __name__ == "__main__":
    unittest.main()
