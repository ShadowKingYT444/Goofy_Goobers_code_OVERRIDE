"""Validate the actual linked firmware after `pros make clean && pros make`.

This catches missing competition entry points and invalid constructor targets;
it cannot replace a boot test on the V5 Brain.
"""

from pathlib import Path
import struct
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]


class FirmwareImageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.elf = (ROOT / "bin/monolith.elf").read_bytes()
        # ELF32 little-endian ARM image, with 40-byte section headers.
        assert cls.elf[:6] == b"\x7fELF\x01\x01"
        assert struct.unpack_from("<H", cls.elf, 18)[0] == 40
        offset = struct.unpack_from("<I", cls.elf, 32)[0]
        size, count, names_index = struct.unpack_from("<HHH", cls.elf, 46)
        headers = [struct.unpack_from("<10I", cls.elf, offset + i * size)
                   for i in range(count)]
        names_header = headers[names_index]
        names = cls.elf[names_header[4]:names_header[4] + names_header[5]]
        cls.sections = {}
        for header in headers:
            name = names[header[0]:].split(b"\0", 1)[0].decode()
            cls.sections[name] = header
        cls.symbols = subprocess.check_output(
            ["arm-none-eabi-nm", "-C", str(ROOT / "bin/monolith.elf")],
            text=True,
        )

    def executable(self, address):
        address &= ~1  # Thumb function pointers set bit zero.
        return any(h[2] & 4 and h[3] <= address < h[3] + h[5]
                   for h in self.sections.values())

    def test_single_image_upload(self):
        self.assertGreater((ROOT / "bin/monolith.bin").stat().st_size, 0)
        # The CLI selects the split image if both packages exist and are newer.
        self.assertFalse((ROOT / "bin/hot.package.bin").exists())
        self.assertFalse((ROOT / "bin/cold.package.bin").exists())

    def test_competition_entry_points(self):
        for name in ("initialize", "autonomous", "opcontrol", "disabled",
                     "competition_initialize"):
            entry = next(line for line in self.symbols.splitlines()
                         if line.endswith(" T " + name))
            self.assertTrue(self.executable(int(entry.split()[0], 16)), name)

    def test_constructor_targets(self):
        header = self.sections[".init_array"]
        contents = self.elf[header[4]:header[4] + header[5]]
        self.assertGreater(len(contents), 0)
        for (address,) in struct.iter_unpack("<I", contents):
            self.assertTrue(self.executable(address), hex(address))

    def test_unused_selector_is_not_linked(self):
        self.assertNotIn("ts::selector", self.symbols)
        self.assertNotIn("_GLOBAL__sub_I_registry_internal", self.symbols)

    def test_no_external_tracking_wheels(self):
        for name in ("vertical_odom", "horizontal_odom", "vertical_wheel",
                     "horizontal_wheel"):
            self.assertNotIn(" " + name + "\n", self.symbols)
        for name in ("lift_sensor", "claw_sensor", "imu"):
            self.assertIn(" " + name + "\n", self.symbols)


if __name__ == "__main__":
    unittest.main()
