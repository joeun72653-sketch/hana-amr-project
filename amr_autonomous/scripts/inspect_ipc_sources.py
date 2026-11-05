#!/usr/bin/env python3
"""Check which IPC real_amr_lidar source files exist without changing the originals.

Optional --copy-for-review copies an allowlisted set to ignored local
amr_autonomous/.ipc_src_review/ (never commits/pushes or exposes contents).
Review private/hardware information before any public GitHub publication.
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

CANDIDATES = (
    "launch/real_amr_sensor_qr_test.launch.py",
    "launch/real_amr_slam_mapping.launch.py",
    "launch/real_mapex_nav2.launch.py",
    "launch/real_mapex_autonomous_mapping.launch.py",
    "scripts/gls611_pose_node.py",
    "scripts/qr_sensor_test_geometry.py",
    "scripts/real_dual_lidar_scan_merge.py",
    "scripts/real_lidar_icp_odometry.py",
    "scripts/real_mapex_cmd_vel_to_rpm.py",
    "scripts/real_mapex_wheel_odometry.py",
    "package.xml",
    "setup.py",
    "setup.cfg",
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-root", type=Path,
        default=Path.home() / "real_amr/src/real_amr_lidar",
        help="Local package root on Ubuntu IPC",
    )
    parser.add_argument(
        "--copy-for-review", action="store_true",
        help="Copy allowlisted files to the Git-ignored local review folder",
    )
    args = parser.parse_args()

    source = args.source_root.expanduser().resolve()
    review = Path(__file__).resolve().parents[1] / ".ipc_src_review"
    present = missing = copied = 0

    print(f"[IPC] source={source}")
    for rel in CANDIDATES:
        path = source / rel
        if not path.is_file():
            missing += 1
            print(f"  MISSING {rel}")
            continue

        present += 1
        print(f"  FOUND   {rel} ({path.stat().st_size} bytes)")
        if args.copy_for_review:
            dest = review / rel
            if dest.exists():
                print(f"  SKIP    {rel} (review file already exists)")
            else:
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, dest)
                copied += 1

    print(f"[IPC] present={present} missing={missing} staged_for_review={copied}")
    if args.copy_for_review:
        print(f"[IPC] REVIEW ONLY, NOT UPLOADED: {review}")
        print("[IPC] Check private identifiers, credentials and license before publication.")
    return 0 if present else 2


if __name__ == "__main__":
    raise SystemExit(main())
