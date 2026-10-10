from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "fit_gps_mount.py"
spec = importlib.util.spec_from_file_location("fit_gps_mount", MODULE_PATH)
assert spec is not None and spec.loader is not None
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


class FitGpsMountTests(unittest.TestCase):
    @staticmethod
    def samples(forward: float, right: float, corrupt: bool = False):
        result = []
        for index, heading in enumerate(range(0, 360, 45), start=1):
            angle = math.radians(heading)
            x = 12.0 + forward * math.sin(angle) + right * math.cos(angle)
            y = -8.0 + forward * math.cos(angle) - right * math.sin(angle)
            x += 0.08 if index % 2 else -0.08
            y += 0.05 if index % 3 else -0.05
            if corrupt and index == 4:
                x += 3.0
                y -= 2.0
            result.append(module.Sample(index, x, y, heading))
        return result

    def test_recovers_known_offset(self):
        fit = module.fit_mount(self.samples(3.0, 3.7))
        self.assertAlmostEqual(fit.forward_in, 3.0, delta=0.1)
        self.assertAlmostEqual(fit.right_in, 3.7, delta=0.1)
        self.assertLess(fit.rms_in, 0.15)
        self.assertEqual(module.quality(fit)[0], "PASS")

    def test_detects_center_movement_or_bad_stop(self):
        fit = module.fit_mount(self.samples(3.0, 3.7, corrupt=True))
        verdict, issues = module.quality(fit)
        self.assertNotEqual(verdict, "PASS")
        self.assertTrue(issues)
        self.assertGreater(fit.max_residual_in, 2.0)

    def test_parses_serial_and_plain_rows(self):
        parsed = module.parse_samples(
            [
                "GPS_MOUNT_SAMPLE,1,12.0,-4.0,0.0",
                "2,14.0,-3.0,45.0",
                "15.0,-2.0,90.0",
            ]
        )
        self.assertEqual(len(parsed), 3)
        self.assertEqual(parsed[0].index, 1)
        self.assertEqual(parsed[2].index, 3)
        self.assertEqual(parsed[2].heading_deg, 90.0)

    def test_rejects_narrow_heading_coverage(self):
        samples = [module.Sample(i, float(i), 0.0, float(i * 5)) for i in range(8)]
        with self.assertRaisesRegex(ValueError, "full 360"):
            module.fit_mount(samples)


if __name__ == "__main__":
    unittest.main()
