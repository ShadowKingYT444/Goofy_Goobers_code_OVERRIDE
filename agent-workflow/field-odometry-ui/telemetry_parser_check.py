import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SERVER_PATH = ROOT / "tools" / "lidar_bar_server.py"


def load_server_module():
    spec = importlib.util.spec_from_file_location("lidar_bar_server", SERVER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    server = load_server_module()

    old_frame = server.parse_d4(
        "D4 s=1 t=20 p6=100,63,1 p7=101,63,1 p8=102,63,1 p9=103,63,1 "
        "m17=inf m18=10.0 m11=-20.0 m13=-20.0 errno=0"
    )
    assert old_frame["motors"]["18"]["position_deg"] == 10.0
    assert "17" not in old_frame["motors"]
    assert old_frame["odometer"] == {}

    new_frame = server.parse_d4(
        "D4 s=2 t=40 p6=200,63,1 p7=201,63,1 p8=202,63,1 p9=203,63,1 "
        "m17=0.0 m18=10.0 m11=-20.0 m13=-20.0 h5=12345 errno=0"
    )
    assert new_frame["odometer"]["5"]["position_centideg"] == 12345

    phase_pose = server.parse_fuse_test(
        "FUSE_TEST phase=drive_to_wp1 traveled=3.20 target=12.00 remaining=8.80 "
        "power=95 turn=-3 x=1.25 y=4.50 h=88.40 imu=1.60 lidar=left reject=none"
    )
    assert phase_pose["phase"] == "drive_to_wp1"
    assert abs(phase_pose["x"] - 1.25) < 0.001
    assert abs(phase_pose["y"] - 4.50) < 0.001
    assert abs(phase_pose["heading_deg"] - 88.40) < 0.001
    assert abs(phase_pose["traveled"] - 3.20) < 0.001
    assert phase_pose["lidar"] == "left"

    snapshot_pose = server.parse_fuse_test(
        "FUSE_TEST phase=after_turn x=4.00 y=11.50 heading=45.00 imu=45.00 bias=0.00 lidar=none reject=wait"
    )
    assert snapshot_pose["phase"] == "after_turn"
    assert abs(snapshot_pose["heading_deg"] - 45.00) < 0.001
    assert abs(snapshot_pose["bias"] - 0.00) < 0.001

    concatenated_pose = server.parse_fuse_test(
        "soutFUSE_TEST phase=telemetry x=-48.00 y=0.00 heading=86.40 "
        "imu=90.00 bias=-3.60 lidar=audience reject=wait theta=-3.60 "
        "distance=17.25 rmse=0.05 soutD4 s=9 t=180 p6=430,63,1 "
        "p7=430,63,1 p8=430,63,1 p9=430,63,1 imu=0.00 rawimu=0.00"
    )
    assert concatenated_pose["phase"] == "telemetry"
    assert abs(concatenated_pose["imu"] - 90.00) < 0.001
    assert abs(concatenated_pose["theta"] + 3.60) < 0.001
    assert concatenated_pose["reject"] == "wait"

    print("telemetry parser checks passed")


if __name__ == "__main__":
    main()
