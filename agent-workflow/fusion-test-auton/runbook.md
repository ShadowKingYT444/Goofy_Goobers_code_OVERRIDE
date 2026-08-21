Run source check:

```powershell
.\.venv\Scripts\python.exe .\agent-workflow\fusion-test-auton\fusion_test_source_check.py
```

Build:

```powershell
.\.venv\Scripts\pros.exe --version
.\.venv\Scripts\pros.exe lsusb --target v5
.\.venv\Scripts\python.exe -m pros.cli.main make
```

Upload, if build succeeds and the Brain is connected:

```powershell
.\.venv\Scripts\python.exe -m pros.cli.main upload --slot 1 --name MoreVex --description MoreVex --after none
```

Run from controller:

```text
Press X + Down.
Brain line 6 should show: X+Down real fusion
```

Expected movement:

```text
drive to wp1, about 12 in forward from the start pose
turn to start heading - 45 deg
drive to wp2, about 12 in along the new field heading
```

Expected console markers:

```text
FUSE_TEST phase=start
FUSE_TEST route=start
FUSE_TEST command=fused_drive_wp1 type=fused_drive_to
FUSE_TEST phase=fused_drive_wp1 controller=fused_drive
FUSE_TEST phase=after_first
FUSE_TEST command=fused_turn_45cw type=fused_turn_to
FUSE_TEST phase=fused_turn_45cw controller=fused_turn
FUSE_TEST phase=after_turn
FUSE_TEST command=fused_drive_wp2 type=fused_drive_to
FUSE_TEST phase=fused_drive_wp2 controller=fused_drive
FUSE_TEST phase=final
FUSE_TEST imu_correction ...
```

Key values to watch:

```text
fused_drive dist should shrink toward 0
fused_drive bearing_err should stay controlled instead of growing
fused_turn error should shrink toward 0
fused movement may show imu_correction mode=bias on clean wall readings
heading should approach 45 deg without bouncing back to 10-30 deg
```
